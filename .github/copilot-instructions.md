# GitHub Copilot Instructions – Team 2556 FRC 2026 Robot

This is the RobotPy codebase for **Team 2556 Radioactive Roaches** for the 2026 FRC season. It uses the **WPILib Commands2** framework in Python.

---

## Project Structure

| Directory | Purpose |
|-----------|---------|
| `commands/` | WPILib Command classes (one action or sequence per file) |
| `constants/` | Robot-wide constants (CAN IDs, PID values, geometry) |
| `subsystems/` | WPILib Subsystem classes (hardware wrappers) |
| `util/` | Shared helper classes (controller shaping, math, NT helpers) |
| `resources/` | Elastic dashboard layouts, controller diagrams |
| `tests/` | pyfrc unit and simulation tests |

---

## Key Hardware & Libraries

- **Motor controllers**: CTRE Phoenix 6 (`phoenix6`) – TalonFX for drive, steer, shooter, intake, transfer, hood, and climb
- **Sensors**: Pigeon 2 IMU (CAN 13), CANcoder absolute encoders (CAN 9-12)
- **Vision**: Limelight cameras (`util/limelight_helpers.py`) – AprilTag odometry
- **LEDs**: CANdle controller (CAN 32) via `subsystems/led/`
- **Framework**: `commands2`, `wpilib`, `wpimath`

### CAN ID Map (from `constants/canbus.py`)
- **Drivetrain**: Drive 1–4, Steer 5–8, CANcoder 9–12, Pigeon 13
- **Intake**: Roller 16, Left pivot 17, Right pivot 18
- **Transfer**: Spindexer 21, Up-transfer 22
- **Shooter**: Bottom 26, Top 27, Hood 28
- **Climb**: Motor 41
- **CANdle**: 32

---

## Code Conventions

### Commands
- Extend `commands2.Command`
- Override `initialize()`, `execute()`, `isFinished()`, and `end(cancelled)`
- Constructor receives subsystem(s) and calls `self.addRequirements(subsystem)`
- File naming: `snake_case.py`, class naming: `PascalCaseCommand`

### Subsystems
- Extend `commands2.SubsystemBase`
- Hardware objects created in `__init__` using CAN IDs from `constants/canbus.py`
- Override `periodic()` for telemetry / state updates
- Publish diagnostics to NetworkTables with `wpilib.SmartDashboard` or `ntcore`

### Constants
- Plain Python class with class-level attributes – no instances
- Group by hardware system, one file per system in `constants/`
- Use `from constants.canbus import kCanbus` pattern for imports

### Controller Input Shaping
- Use `util/custom_controller.py` `XboxController` instead of raw `CommandXboxController`
- Chain `.with_deadband(0.3).with_power(5).with_mult(0.6)` for each controller
- All joystick getters automatically apply: deadband → power curve → multiplier → clamp

---

## LED System

States are created and managed by `CANdleLEDController` in `subsystems/led/LED_controller.py`:

```python
# Create a named state (higher priority wins)
led_controller.create_state(
    "my_state_key",           # unique key
    ColorFactories.strobe(CANdle_Color.RED),  # animation
    priority=10,              # higher = overrides lower
    enable=True               # set False to hide without deleting
)

# Disable/remove a state
led_controller.create_state("my_state_key", ..., enable=False)
```

Use `subsystems/led/LED_helpers.py` `ColorFactories` for animations: `solid_color`, `strobe`, `fire`, `rainbow`, `larson`, `color_flow`, `rgb_fade`, `single_fade`, `twinkle`, `twinkle_off`.

---

## Vision / Odometry

`subsystems/vision/mono_limelight.py` `Vision` accepts multiple camera names and picks the best measurement each cycle:
- Selects camera with most AprilTags; tiebreaks by closest average distance
- Rejects measurements when angular velocity is high or robot tilt exceeds threshold
- Supports MegaTag2 (`orb_wpiblue`) and standard `botpose_wpiblue`

---

## Test Program

`commands/test_program.py` `TestProgramCommand` runs 10 sequential phases at robot startup in **Test mode**:
1. Idle
2. IMU tilt check (Pigeon 2 pitch + roll)
3. LED color cycle
4. Drivetrain forward movement (encoder validation)
5. Swerve steer motor angle test
6. Shooter spin-up RPM test
7. Hood position test
8. Intake deployer test
9. Transfer motor test
10. Results summary (rainbow = all pass, yellow = partial, red = fail)

Safety timeouts: hazard motors (hood CAN 28, intake CAN 17/18, transfer CAN 21/22) → 0.5 s max; shooter (CAN 26/27) → 2.0 s max.

---

## Robot Init Flow

```
robot.py MyRobot
  └── robotInit()     → creates RobotContainer
  └── autonomousInit()→ schedules getAutonomousCommand()
  └── teleopInit()    → cancels autonomous
  └── testInit()      → schedules TestProgramCommand

robotcontainer.py RobotContainer
  └── __init__()      → creates all subsystems + controllers
  └── configureButtonBindings() → wires commands to buttons
  └── getAutonomousCommand()    → returns auto command
```

---

## Adding New Subsystems

1. Create `subsystems/<name>/<name>_subsystem.py` extending `commands2.SubsystemBase`
2. Add CAN IDs to `constants/canbus.py`
3. Add tuning constants to `constants/<name>.py`
4. Instantiate in `RobotContainer.__init__()` and pass to commands

## Adding New Commands

1. Create `commands/<category>/<command_name>.py` extending `commands2.Command`
2. Import subsystem(s) in constructor and call `self.addRequirements()`
3. Bind to a button in `RobotContainer.configureButtonBindings()`
