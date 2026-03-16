# REV PDH + Per-Motor Power Monitoring — Battery Life Extension

## Context
The REV Power Distribution Hub (PDH, CAN ID 31) is on the CAN bus but completely unused in code. Additionally, every TalonFX/TalonFXS already reports stator current, supply current, and supply voltage for free (no Pro license needed). By combining PDH system-level data with per-motor current from Phoenix 6, we can build a full power budget view.

**Goals:**
1. **Dashboard telemetry** — drivers/pit crew see live power data on Elastic
2. **Low-voltage warnings** — LED color change when battery is sagging
3. **Per-motor current breakdown** — identify which subsystem draws the most
4. **Adaptive speed limiting** — automatically reduce drivetrain speed when voltage drops to prevent brownouts

**Free Phoenix 6 signals (no Pro license):**
| Signal | Method | Returns |
|---|---|---|
| Stator Current | `motor.get_stator_current().value` | Amps through motor windings |
| Supply Current | `motor.get_supply_current().value` | Amps drawn from battery |
| Supply Voltage | `motor.get_supply_voltage().value` | Volts at the controller |
| Motor Voltage | `motor.get_motor_voltage().value` | Volts applied to motor |

**What requires Pro (we skip):** FOC, SignalLogger hoot-file recording.

**PDH API (`wpilib.PowerDistribution`):**
- `getVoltage()` — battery voltage
- `getTotalCurrent()` — sum of all channels
- `getCurrent(channel)` — per-PDH-channel current (0-23)
- `getTotalEnergy()` / `getTemperature()` — return 0 on REV PDH, skip these

---

## Files to Create/Modify

1. **`util/power_monitor.py`** (NEW) — PowerMonitor subsystem
2. **`robotcontainer.py`** — instantiate PowerMonitor, wire voltage-based speed limiting + LED
3. **`resources/elastic-layout-v2.json`** — add power telemetry widgets

---

## Implementation

### 1. `util/power_monitor.py` (NEW)

A subsystem that wraps both the PDH and per-motor current readings.

```python
import wpilib
from wpilib import PowerDistribution
from commands2 import Subsystem
from util.nt_util import NTTable
from constants.canbus import kCANId


class PowerMonitor(Subsystem):
    # Voltage thresholds
    WARNING_VOLTAGE = 11.0      # amber LED warning
    CRITICAL_VOLTAGE = 10.0     # red LED + speed reduction
    SPEED_LIMIT_VOLTAGE = 10.5  # start tapering speed below this

    def __init__(self):
        self.pdh = PowerDistribution(
            kCANId.PDH, PowerDistribution.ModuleType.kRev
        )

        self.nt = NTTable("Power")
        self.nt.float("Battery Voltage", 0.0)
        self.nt.float("Total Current (A)", 0.0)
        self.nt.float("Total Power (W)", 0.0)
        self.nt.float("Min Voltage", 13.0)
        self.nt.string("Status", "OK")

        # Per-subsystem current tracking (populated via register_motors)
        self._motor_groups = {}  # name -> list of motors
        self._min_voltage = 13.0
        self._voltage = 12.5

    def register_motors(self, group_name: str, motors: list):
        """Register a group of motors (e.g. 'Drivetrain', 'Shooter') for current tracking.
        Call from robotcontainer after subsystems are created."""
        self._motor_groups[group_name] = motors
        self.nt.float(f"{group_name} Current (A)", 0.0)

    @property
    def voltage(self) -> float:
        return self._voltage

    @property
    def is_low_voltage(self) -> bool:
        return self._voltage < self.SPEED_LIMIT_VOLTAGE

    def get_speed_scale(self) -> float:
        """Returns 0.5-1.0 scale factor based on voltage. 1.0 above SPEED_LIMIT_VOLTAGE,
        linearly tapers to 0.5 at CRITICAL_VOLTAGE."""
        if self._voltage >= self.SPEED_LIMIT_VOLTAGE:
            return 1.0
        if self._voltage <= self.CRITICAL_VOLTAGE:
            return 0.5
        ratio = (self._voltage - self.CRITICAL_VOLTAGE) / (self.SPEED_LIMIT_VOLTAGE - self.CRITICAL_VOLTAGE)
        return 0.5 + 0.5 * ratio

    def periodic(self):
        # PDH system-level readings
        self._voltage = self.pdh.getVoltage()
        total_current = self.pdh.getTotalCurrent()

        if self._voltage < self._min_voltage:
            self._min_voltage = self._voltage

        # Status string
        if self._voltage < self.CRITICAL_VOLTAGE:
            status = "CRITICAL"
        elif self._voltage < self.WARNING_VOLTAGE:
            status = "WARNING"
        else:
            status = "OK"

        self.nt.set("Battery Voltage", round(self._voltage, 2))
        self.nt.set("Total Current (A)", round(total_current, 1))
        self.nt.set("Total Power (W)", round(self._voltage * total_current, 1))
        self.nt.set("Min Voltage", round(self._min_voltage, 2))
        self.nt.set("Status", status)

        # Per-motor-group supply current (free Phoenix 6 signal)
        for group_name, motors in self._motor_groups.items():
            group_current = sum(
                abs(m.get_supply_current().value) for m in motors
            )
            self.nt.set(f"{group_name} Current (A)", round(group_current, 1))
```

Key design choices:
- `register_motors()` lets robotcontainer pass motor references without the monitor importing subsystems
- Uses `get_supply_current()` (battery-side amps) not stator current — shows actual battery drain
- Groups motors by subsystem name so dashboard shows "Drivetrain Current", "Shooter Current", etc.
- Only reads supply current (1 CAN frame per motor per cycle) — lightweight

---

### 2. `robotcontainer.py` — Wire it up

Add import:
```python
from util.power_monitor import PowerMonitor
```

In `__init__`, after subsystem creation:
```python
self.power_monitor = PowerMonitor()

# Register motor groups for per-subsystem current tracking
# Drivetrain motors are inside the CTRE swerve module — access via the generated module list
self.power_monitor.register_motors("Intake", [
    self.intake_subsystem.left_deployer,
    self.intake_subsystem.right_deployer,
    self.intake_subsystem.spinny_motor,
])
self.power_monitor.register_motors("Shooter", [
    self.shooter_subsystem.bottom_motor,
    self.shooter_subsystem.top_motor,
])
self.power_monitor.register_motors("Hood", [
    self.hood_subsystem.hood_motor,
])
self.power_monitor.register_motors("Transfer", [
    self.transfer_subsystem.spindex_motor,
    self.transfer_subsystem.up_transfer_motor,
])
```

> **Note:** Drivetrain motors are managed by CTRE's swerve module and may not be directly accessible. If they are, add them too. If not, the PDH total minus the other groups gives a rough drivetrain estimate.

Voltage-based speed limiting trigger:
```python
Trigger(lambda: self.power_monitor.is_low_voltage).whileTrue(
    cmd.run(lambda: self._drivetrain.change_speed_mult(
        kDriveConfig.SPEED_MULT * self.power_monitor.get_speed_scale(),
        kDriveConfig.ROTATION_MULT * self.power_monitor.get_speed_scale(),
    ))
)
```

LED warning triggers:
```python
Trigger(lambda: self.power_monitor.voltage < PowerMonitor.WARNING_VOLTAGE).whileTrue(
    InstantCommand(lambda: self.LED_controller.set_color("battery_low", AMBER))
)
Trigger(lambda: self.power_monitor.voltage < PowerMonitor.CRITICAL_VOLTAGE).whileTrue(
    InstantCommand(lambda: self.LED_controller.set_color("battery_critical", RED))
)
```

---

### 3. `resources/elastic-layout-v2.json` — Dashboard widgets

Add a "Power" section with:
- Battery Voltage (number display)
- Total Current (A)
- Total Power (W)
- Min Voltage (number display)
- Status (text display: OK / WARNING / CRITICAL)
- Per-subsystem current bars:
  - Drivetrain Current (A) — if available
  - Intake Current (A)
  - Shooter Current (A)
  - Hood Current (A)
  - Transfer Current (A)

---

## Battery Life Extension Strategy

| Feature | How it helps |
|---|---|
| **Battery voltage telemetry** | Pit crew can see battery health and swap before a match if sagging |
| **Min voltage tracking** | Post-match: if min voltage < 10V, battery needs charging or retiring |
| **Adaptive speed limiting** | Auto-reduces drivetrain speed (biggest current draw) when voltage drops, preventing brownout resets |
| **LED warnings** | Drivers see amber/red when battery is low — adapt driving style |
| **Per-subsystem current** | Identify which motors eat the most power; tune current limits accordingly |
| **Total current display** | Spot abnormally high draw (stuck motor, shorted wire) immediately |

---

## Per-Motor Current vs PDH Channels

Two complementary data sources:

| Source | Pros | Cons |
|---|---|---|
| **PDH `getTotalCurrent()`** | Single call, system-wide total | No per-motor breakdown |
| **PDH `getCurrent(ch)`** | Per-channel (wire-level) | 24 CAN frames if reading all; need channel-to-motor mapping |
| **Phoenix 6 `get_supply_current()`** | Per-motor, already on CAN bus, free | Only for CTRE motors (not servos, sensors, etc.) |

**Our approach:** Use PDH for system total + voltage. Use Phoenix 6 `get_supply_current()` for per-motor breakdown. Skip PDH per-channel reads (heavy CAN load, channel mapping is fragile).

---

## What we intentionally skip

- **PDH per-channel current (`getAllCurrents()`)**: 24 CAN frames per cycle, and channel-to-motor mapping is fragile. Per-motor Phoenix 6 signals are better.
- **Energy tracking**: `getTotalEnergy()` returns 0 on REV PDH.
- **Temperature**: `getTemperature()` returns 0 on REV PDH.
- **Switchable channel**: Not needed unless we have a device on it.
- **SignalLogger / hoot files**: Requires Pro license.

---

## Verification
- Deploy and check Elastic dashboard for Power section with live voltage/current
- Verify per-subsystem currents update when each mechanism runs (spin shooter, deploy intake, etc.)
- Pull battery low (drive hard into a wall) — voltage should dip and status should change
- Verify speed taper kicks in below 10.5V (can test by lowering threshold temporarily)
- Verify LED changes color at warning/critical thresholds
- Check that CAN bus isn't overloaded (no new "CAN overflow" errors in DS log)
- Post-match: check Min Voltage reading to assess battery health
