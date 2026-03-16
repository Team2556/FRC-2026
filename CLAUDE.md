# FRC 2026 Prototype — Claude Code Guidelines

This project is a **RobotPy (Python) FRC robot** using the **commands2 command-based framework**, **Phoenix 6 (CTRE TalonFX)**, and **PathPlannerLib**.

---

## Project Structure

```
robot.py              # Entry point — minimal lifecycle methods only
robotcontainer.py     # Subsystem + command wiring, button bindings
pyproject.toml        # RobotPy/pip dependency management
constants/            # Per-subsystem constant files (k-prefix classes)
subsystems/           # One file (or subdirectory) per subsystem
commands/             # One file per command
util/                 # Shared utilities (controllers, math helpers)
tests/                # pyfrc tests
```

---

## Mandatory Rules — Commands

### Always call `super().__init__()` first
Every `commands2.Command` subclass **must** call `super().__init__()` as the very first line of `__init__`. Missing this prevents the command from registering with the scheduler.

```python
class MyCommand(commands2.Command):
    def __init__(self, subsystem: MySubsystem):
        super().__init__()   # REQUIRED — must be first
        self._subsystem = subsystem
        self.addRequirements(subsystem)
```

### Always declare requirements
Every command that uses a subsystem **must** call `self.addRequirements(subsystem)`. Without this, two commands can simultaneously control the same hardware with no scheduler protection.

### All four lifecycle methods must be present
```python
def initialize(self):   # Called once when scheduled
    pass

def execute(self):      # Called every 20ms while running
    pass

def isFinished(self) -> bool:   # Return True to end naturally
    return False

def end(self, interrupted: bool):   # ALWAYS stop hardware here
    self._subsystem.stop()
```

- `end()` **must** accept `interrupted: bool`.
- Always stop motors/actuators in `end()` regardless of the `interrupted` flag.
- `isFinished()` returning `False` means "run until interrupted" — correct for default commands and `whileTrue()` bindings.

### Do NOT set InterruptionBehavior as an instance attribute
This is **wrong** — it does nothing at runtime:
```python
# WRONG
self.InterruptionBehavior = commands2.InterruptionBehavior.kCancelIncoming
```
Override the method instead:
```python
# CORRECT
def getInterruptionBehavior(self):
    return commands2.InterruptionBehavior.kCancelIncoming
```

---

## Mandatory Rules — Subsystems

### Always call `super().__init__()` first
Every `commands2.Subsystem` subclass **must** call `super().__init__()` as the very first line of `__init__`. Missing this means the subsystem never registers with the scheduler.

```python
class MySubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()   # REQUIRED — must be first
```

### `periodic()` is for telemetry only — not hardware control
`periodic()` is called every 20ms by the scheduler in all modes. Use it **only** for:
- SmartDashboard / NetworkTables telemetry updates
- Odometry updates
- Sensor reads for state estimation

Do **not** drive motors, actuators, or set control outputs inside `periodic()`. Hardware control belongs in `command.execute()`.

### Expose public methods for hardware actions
Commands should call named methods on the subsystem. Do not access private subsystem fields from commands.

```python
# WRONG — accessing private internals from a command
self._drivetrain._drivetrain.seed_field_centric()

# CORRECT — subsystem exposes a public method
self._drivetrain.seed_field_centric()
```

---

## Mandatory Rules — RobotContainer

- `RobotContainer.__init__()` order: instantiate controllers → instantiate subsystems → call `self.configureButtonBindings()`.
- Use `.whileTrue()` for "hold to activate" behaviors (most common).
- Use `.onTrue()` for "press once, runs to completion" behaviors.
- Set default commands with `subsystem.setDefaultCommand(command)` inside `configureButtonBindings()`.
- `getAutonomousCommand()` must **return** the autonomous command — do not put `pass` before the return code, as Python returns `None` at the first `pass` and all following code becomes unreachable.

---

## Mandatory Rules — robot.py

Keep `robot.py` minimal. The only code that belongs here:
- `robotInit()`: instantiate `RobotContainer`
- `robotPeriodic()`: run `CommandScheduler` (already handled by `TimedCommandRobot`)
- `autonomousInit()`: schedule the autonomous command
- `teleopInit()`: cancel the autonomous command
- `testInit()`: cancel all commands

Do **not** add robot logic to `teleopPeriodic()`, `autonomousPeriodic()`, or `disabledPeriodic()`. Logic belongs in subsystems and commands.

---

## Constants

### Rules
- Constants live in `constants/` with one file per subsystem area.
- Use `k`-prefix class names: `kDriveConfig`, `kAutoAlign`, `kHub`, etc.
- Constants are plain class attributes — **no `__init__`**, no instantiation.
- Never mutate constants at runtime. If a value needs to be tunable (e.g., PID gains from SmartDashboard), track the mutable value as subsystem instance state, not in the constants class.
- Never import with `from constants.x import *` — always import the class directly for traceability.

### Mutable objects in constants — avoid this pattern
`PIDController`, `Translation2d`, and similar mutable objects stored as class-level constants are shared across all users. Calling `.enableContinuousInput()` or modifying them mutates the shared object.

```python
# RISKY — kAutoAlign.ROTATIONAL_PID is a shared mutable object
class kAutoAlign:
    ROTATIONAL_PID = PIDController(5.0 / 180.0, 0, 0)

# SAFER — instantiate a fresh controller in the command that needs it
self.rotation_PID = PIDController(kAutoAlign.ROTATIONAL_KP, 0, 0)
self.rotation_PID.enableContinuousInput(-180.0, 180.0)
```

---

## Command Composition

Prefer built-in composition over writing custom multi-step command classes:

```python
# Sequential
cmd1.andThen(cmd2).andThen(cmd3)
commands2.SequentialCommandGroup(cmd1, cmd2, cmd3)

# Parallel (ends when all finish)
cmd1.alongWith(cmd2)
commands2.ParallelCommandGroup(cmd1, cmd2)

# Race (ends when first finishes)
cmd1.raceWith(cmd2)

# Timeout
cmd.withTimeout(2.0)

# Wait
commands2.WaitCommand(1.0)

# Conditional at schedule time
commands2.cmd.either(cmd_true, cmd_false, condition_callable)
```

---

## Phoenix 6 (CTRE) Patterns

- Motor configuration is applied via `motor.configurator.apply(config)` — retry on failure (up to 5 attempts).
- Use `phoenix6.controls.VelocityVoltage`, `DutyCycleOut`, `PositionVoltage`, etc. for control requests — do not use `motor.set()` for final robot code (it is only valid for open-loop duty cycle).
- TalonFX CAN bus string: `"rio"` for the default CAN bus; `"Canivore"` if using a CANivore.
- Read sensor values with `motor.get_velocity().value`, `motor.get_position().value` — `.value` is required to get the float.
- Swerve drive: use `swerve.requests.FieldCentric()` for field-relative driving; call `.set_control(request)` on the `SwerveDrivetrain` object.

---

## PathPlannerLib Patterns

- Paths are defined in the PathPlanner GUI and saved to `deploy/pathplanner/`.
- Load named paths with `PathPlannerPath.fromPathFile("PathName")`.
- Use `AutoBuilder.followPath(path)` to generate a command for a path.
- `AutoBuilder` must be configured once in the drivetrain subsystem `__init__` before any auto commands are created.
- Alliance reflection: use `.with_reflected_red_alliance_pose()` on `DriveToASpot` for custom spot-to-spot commands, or use `PathPlannerPath.flipPath()` for PathPlanner paths.

---

## Python-Specific Rules

- **Type hints**: always add type hints to `__init__` parameters for subsystem arguments. This is required for IDE support.
- **`match` statements**: Python 3.12 is available on the roboRIO — `match`/`case` syntax is safe to use (see `util/alliance_constant.py`).
- **Protected convention**: prefix private/internal attributes with `_` (e.g., `self._drivetrain`, `self._stop()`).
- **Do not use `global`**: all subsystem state should be instance attributes, not module-level globals.
- **Builder/fluent pattern**: methods that configure a command should return `self` to enable chaining (e.g., `with_precise_values()`, `with_deadband()`).
- **SmartDashboard writes are slow**: limit `SmartDashboard.put*()` calls to `periodic()` methods. Do not call them inside `execute()` on fast command loops.

---

## Dependency Management

Dependencies are declared in `pyproject.toml`:

```toml
[tool.robotpy]
robotpy_version = "2026.2.1"
components = ["commands2"]
requires = [
    "phoenix6~=26.1",
    "robotpy-pathplannerlib",
    "robotpy-apriltag",
]
```

- `components`: official RobotPy extras (from `pip install robotpy[...]`).
- `requires`: arbitrary PyPI/vendor packages.
- Deploy: `python -m robotpy deploy`
- Sync (download deps): `python -m robotpy sync`
- Do not manually edit installed packages — always go through `pyproject.toml`.

---

## Simulation

### Key facts
- `wpilib.simulation` classes are part of the core `wpilib` package — **always available, no extra component needed in `pyproject.toml`**.
- `simulationPeriodic()` is called automatically by the commands2 scheduler only in simulation — no registration needed.
- Only guard the **instantiation** of sim objects with `if wpilib.RobotBase.isSimulation():`. The method itself can be defined unconditionally (the scheduler never calls it on real hardware).
- `FlywheelSim.getAngularVelocity()` returns **rad/s**; Phoenix 6 `set_rotor_velocity()` takes **rotations/s** — convert: `rps = rad_s / (2 * math.pi)`.
- Use `0.02` (20 ms) as the `update()` timestep for non-drivetrain subsystems. Do **not** use a Notifier — that is only for Phoenix 6 swerve odometry (250 Hz).
- Always call `set_supply_voltage(RobotController.getBatteryVoltage())` on `motor.sim_state` before reading `motor_voltage`.

### Velocity-controlled motor (FlywheelSim) pattern
```python
import wpilib
from wpilib import RobotController
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId
import math

class MySubsystem(commands2.Subsystem):
    def __init__(self, motor_type=None, moment_of_inertia=0.001):
        super().__init__()
        self._motor = phoenix6.hardware.TalonFX(id, "rio")
        # ... hardware config ...
        if wpilib.RobotBase.isSimulation():
            _model = motor_type if motor_type is not None else DCMotor.krakenX60(1)
            _plant = LinearSystemId.flywheelSystem(_model, moment_of_inertia, 1.0)
            self._flywheel_sim = FlywheelSim(_plant, _model)

    def simulationPeriodic(self):
        self._motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
        self._flywheel_sim.setInputVoltage(self._motor.sim_state.motor_voltage)
        self._flywheel_sim.update(0.02)
        sim_rps = self._flywheel_sim.getAngularVelocity() / (2 * math.pi)
        self._motor.sim_state.set_rotor_velocity(sim_rps)
        self._motor.sim_state.add_rotor_position(sim_rps * 0.02)
```

### Position-controlled arm (SingleJointedArmSim) pattern
```python
from wpilib.simulation import SingleJointedArmSim
from wpimath.units import degreesToRadians, radiansToDegrees

if wpilib.RobotBase.isSimulation():
    self._arm_sim = SingleJointedArmSim(
        gearbox=DCMotor.krakenX60(1),
        gearing=GEAR_RATIO,         # motor rotations per arm output rotation
        moi=SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, ARM_MASS_KG),
        armLength=ARM_LENGTH_M,
        minAngle=degreesToRadians(MIN_ANGLE_DEG),
        maxAngle=degreesToRadians(MAX_ANGLE_DEG),
        simulateGravity=True,
        startingAngle=degreesToRadians(0.0),
    )
```

### TalonFX limit switch simulation (Phoenix 6 only — do NOT use DIOSim)
Limit switches wired through TalonFX are set directly on `TalonFXSimState`:
```python
tol = degreesToRadians(1.0)
self._motor.sim_state.set_forward_limit(arm_rad >= max_rad - tol)
self._motor.sim_state.set_reverse_limit(arm_rad <= min_rad + tol)
```

### Mechanism2d visualization (Glass)
```python
import wpilib

# In __init__ (inside if isSimulation(): block):
self._mech = wpilib.Mechanism2d(2.0, 2.0)
root = self._mech.getRoot("MyArm", x=1.0, y=0.1)
self._arm_ligament = root.appendLigament("Arm", length=0.4, angle=0.0,
    lineWidth=6.0, color=wpilib.Color8Bit(wpilib.Color.kOrange))
wpilib.SmartDashboard.putData("My Mechanism", self._mech)

# In simulationPeriodic():
self._arm_ligament.setAngle(radiansToDegrees(arm_rad))
```
Open in Glass: NetworkTables → SmartDashboard → My Mechanism → view as Mechanism2d.

### Inertia tuning guidance (ControlledTalonMotor)
Pass `moment_of_inertia=` at the call site in `robotcontainer.py` for per-motor tuning.
Defaults to `0.001` kg·m² (suitable for light rollers/discs). Shooter flywheels typically need `0.005–0.01`.

---

## Known Project Anti-Patterns to Avoid

These are real bugs found in this codebase — do not repeat them:

1. **`pass` before real code in a method** — makes everything after it unreachable (`robotcontainer.py:getAutonomousCommand`).
2. **Missing `super().__init__()`** — `HubAlign` in `commands/auto_align.py` is missing this call.
3. **Accessing private subsystem internals from commands** — `drive_commands.py` calls `self._drivetrain._drivetrain.seed_field_centric()`. Add a public wrapper method to the subsystem instead.
4. **Setting `self.InterruptionBehavior =`** — this is a no-op. Override `getInterruptionBehavior()` instead (`commands/path_commands/drive_to_a_spot.py:63`).
5. **Mutable PIDController in constants** — `kAutoAlign.ROTATIONAL_PID` is shared; calling `.enableContinuousInput()` on it mutates the shared object.
