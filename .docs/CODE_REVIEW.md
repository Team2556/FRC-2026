# Code Review — FRC 2026 Prototype

**Date:** 2026-03-28
**Branch:** `fix/loop-overrun`
**Reviewed against:** CLAUDE.md project rules, WPILib commands2 framework, Phoenix 6 best practices

---

## 1. Critical — Missing `super().__init__()`

Every `Command` and `Subsystem` subclass **must** call `super().__init__()` as the first line of `__init__`. Without it, the class never registers with the command scheduler. **14 classes are affected.**

### Subsystems (4)

| Class | File | Line |
|-------|------|------|
| `TransferSubsystem` | `subsystems/trasnfer/transfer_subsystem.py` | 13 |
| `ShooterHood` | `subsystems/shooter/shooter_hood.py` | 22 |
| `ClimbSubsystem` | `subsystems/climb/climb_subsystem.py` | 17 |
| `Vision` | `subsystems/vision/mono_limelight.py` | 23 |

### Commands (10)

| Class | File | Line |
|-------|------|------|
| `IntakeCommandDeploy` | `commands/intake/intake_commands.py` | 14 |
| `IntakeCommandUndeploy` | `commands/intake/intake_commands.py` | 34 |
| `IntakeForceRetract` | `commands/intake/intake_commands.py` | 59 |
| `IntakeCommandManualForward` | `commands/intake/intake_commands.py` | 80 |
| `IntakeCommandManualReverse` | `commands/intake/intake_commands.py` | 97 |
| `EnableShooter` | `commands/shooter/shooter_commands.py` | 7 |
| `DisableShooter` | `commands/shooter/shooter_commands.py` | 19 |
| `ResetShooterHood` | `commands/shooter/hood_commands.py` | 19 |
| `ManualShooterHood` | `commands/shooter/hood_commands.py` | 42 |
| `UpdateOdometry` | `commands/vision/vision_odometry.py` | 7 |

### Classes that already have it (correct)
`ControllerDrive`, `RunTransferCommand`, `ReverseTransferCommand`, `ClimbUp`, `ClimbDown`, `DriveToASpot`, `TurretTargetBase`, `GoBackWithPath`, `DualMotorShooter`, `IntakeSubsystem`, `CANdleLEDController`, `SwerveDriveTrain`

---

## 2. Missing Command Lifecycle Methods

Per project rules, every command must define all four lifecycle methods: `initialize()`, `execute()`, `isFinished()`, `end(interrupted)`.

| Command | Missing |
|---------|---------|
| `EnableShooter` | `execute()`, `isFinished()` |
| `DisableShooter` | `execute()`, `isFinished()`, `end(interrupted)` |
| `ControllerDrive` | `initialize()`, `isFinished()` |
| `UpdateOdometry` | `initialize()`, `isFinished()`, `end(interrupted)` |
| `ManualShooterHood` | `initialize()`, `isFinished()`, `end(interrupted)` |
| `IntakeCommandDeploy` | `execute()` |
| `IntakeCommandUndeploy` | `execute()` |
| `IntakeCommandManualForward` | `execute()` |
| `IntakeCommandManualReverse` | `execute()` |

**Note:** `DisableShooter` is set as the default command for the shooter subsystem (`robotcontainer.py:88`). Default commands must have `isFinished() -> False` to run continuously.

---

## 3. Mutable Constants — Runtime Mutation

**Rule:** "Never mutate constants at runtime. If a value needs to be tunable, track the mutable value as subsystem instance state."

These subsystems write NT values back onto constant class attributes in `periodic()`:

| Subsystem | File | Constants Mutated |
|-----------|------|-------------------|
| `DualMotorShooter` | `subsystems/shooter/dual_shooter.py:100-102` | `kShooterMotor.IDLE_RPM`, `.TARGET_RPM`, `.REACH_TARGET_VELOCITY_ERROR` |
| `IntakeSubsystem` | `subsystems/intake/intake.py:117-120` | `kIntakePivot.DEPLOYED_POSITION`, `.DEPLOYED_SPEED`, `kIntakeRoller.TARGET_RPM` |
| `TransferSubsystem` | `subsystems/trasnfer/transfer_subsystem.py:71,74` | `kSpindexer.TARGET_RPM`, `kTransfer.TARGET_RPM` |
| `ClimbSubsystem` | `subsystems/climb/climb_subsystem.py:51-52` | `kClimb.POSITION_UP`, `.POSITION_DOWN` |
| `Vision` | `subsystems/vision/mono_limelight.py:100-101` | `kOdometry.MAX_TILT_ERROR`, `.MIN_APRILTAGS` |
| `SwerveDriveTrain` | `subsystems/drivetrain/drivetrain.py:106-107` | `kDriveConfig.SPEED_MULT`, `.ROTATION_MULT` |

### Fix Pattern
Replace constant mutation with instance attributes:
```python
# BEFORE (wrong)
kShooterMotor.TARGET_RPM = self.nt_sub.get('Target RPM')

# AFTER (correct)
self.target_rpm = self.nt_sub.get('Target RPM')
```
Then update all code that reads the constant to use the instance attribute instead.

---

## 4. Private Field Access from Commands

Commands must call public subsystem methods — not reach into private internals.

| Command | File:Line | Violation | Fix |
|---------|-----------|-----------|-----|
| `UpdateOdometry` | `commands/vision/vision_odometry.py:21` | `self._drivetrain._drivetrain.add_vision_measurement(...)` | Add `add_vision_measurement()` public method to `SwerveDriveTrain` |
| `IntakeForceRetract` | `commands/intake/intake_commands.py:69` | `self.intake_subsystem.left_pivot_motor.set(...)` | Add `set_deployer_raw_speed(speed)` public method to `IntakeSubsystem` |

---

## 5. Typos

| What | Where | Current | Should Be |
|------|-------|---------|-----------|
| Folder name | `subsystems/trasnfer/` | `trasnfer` | `transfer` |
| Parameter + field | `commands/transfer/run_transfer_motors.py` (7 occurrences) | `transfer_sybsystem` | `transfer_subsystem` |
| Method name | `commands/auto_align/alignio.py:49` | `pereodic()` | `periodic()` |
| Method call | `commands/auto_align/align_with_controller.py:115` | `self.pereodic()` | `self.periodic()` |
| Variable name | `commands/intake/intake_commands.py:36` | `foward_limit` | `forward_limit` |

**Note:** Renaming the `trasnfer` folder requires updating imports in: `robotcontainer.py`, `run_transfer_motors.py`, `align_with_controller.py`, and any other file that imports from `subsystems.trasnfer`.

---

## 6. Other Issues

### Vision — Duplicate NetworkTables Initialization
`subsystems/vision/mono_limelight.py` lines 28-30 create an NTTable and entries, then lines 32-36 overwrite it with a new NTTable and re-create the same entries. The first block (28-30) should be removed.

### HubAlign — SmartDashboard Read in `execute()`
`commands/auto_align/align_with_controller.py:76` calls `SmartDashboard.getNumber()` every 20ms in `execute()`. Per project rules, SmartDashboard reads/writes should be limited to `periodic()`. Use `NTTable` cached reads or move to a periodic update.

### ConditionalAlignAndShoot — Missing Subsystem Requirement
`commands/auto_align/align_with_controller.py:127-129` calls `self.transfer_subsystem.activate()` and `.stop()` but never calls `self.addRequirements(transfer_subsystem)`. Two commands could simultaneously control the transfer motor.

### Duplicate Button Binding
`robotcontainer.py` lines 157-163: Controller 2 buttons `a()` and `b()` are both bound to `RunTransferCommand` with identical arguments. One is likely meant to be something else (e.g., `ReverseTransferCommand`).

### `getAutonomousCommand()` Returns None
`robotcontainer.py:232` returns `None` with the actual auto chooser call commented out on line 231. The autonomous mode will do nothing until this is restored.

---

## 7. Recommendations from Other FRC RobotPy Teams

Based on reviewing codebases from Lambda-Corps/2025, FRC 2429, HuskieRobotics/3061-lib, Aurobots7456, and official RobotPy/CTRE examples:

### 7.1 Unit Testing with pyfrc
Most competitive Python teams test command logic in isolation using `pyfrc` pytest fixtures. Our `tests/` folder exists but has no tests for commands or subsystem logic. Priority targets:
- Intake deploy/undeploy state transitions
- Shooter state machine (enable/disable/is_charged)
- Auto-align PID convergence

### 7.2 URCL Logging
[robotpy-urcl](https://github.com/robotpy/robotpy-urcl) (from Team 6328) provides automatic non-blocking CAN telemetry capture. Call `urcl.start()` in `robotInit()` and all motor data is recorded to DataLog files viewable in AdvantageScope. Eliminates manual `SignalLogger.write_*` overhead entirely.

### 7.3 DataLogManager
WPILib's `DataLogManager` writes `.wpilog` files without blocking the control loop. Better than SmartDashboard for high-frequency data that doesn't need to be live on the dashboard.

### 7.4 Enum-Based State Machines
The `DualMotorShooter` already uses a clean state machine pattern. Consider applying this consistently to other subsystems — especially intake (deploying/deployed/undeploying/undeployed) and climb (up/down/moving). Replace string-based `self.state` with `enum.Enum`.

### 7.5 Simulation Support
No subsystem currently implements `simulationPeriodic()`. Adding physics models would enable full desktop testing:
- `FlywheelSim` for shooter motors
- `SingleJointedArmSim` for intake deploy mechanism
- `DCMotorSim` for transfer/climb

### 7.6 Type Hints on Public Methods
Add return type hints and parameter types to all public subsystem methods. Improves IDE autocomplete and catches type errors before deploy.

---

## 8. What's Already Good

These patterns are solid and should be maintained:

- **`apply_config()` retry pattern** (`util/config_util.py`) — Follows CTRE's official 5-retry approach
- **`EditablePID` with debounce** (`util/editable_pid.py`) — 1.5s debounce, FMS safety gate, tolerance-based change detection
- **SignalLogger NT toggle** (`subsystems/drivetrain/telemetry.py:20-21, 97`) — Default-off toggle prevents loop overruns
- **Builder/fluent pattern** (`commands/path_commands/drive_to_a_spot.py`) — `with_precise_values()`, `with_parallel_command()` return `self`
- **State machine** (`subsystems/shooter/dual_shooter.py`) — Clean enum-driven state with `periodic()` applying appropriate control
- **NTTable utility** (`util/nt_util.py`) — Cached entry lookups, type-safe wrappers
- **Explicit imports** — No `from constants.x import *` anywhere
- **`addRequirements()`** — Present on most commands (exception noted in Section 6)

---

## Reference Projects

| Team | Repo | Notable Pattern |
|------|------|-----------------|
| Lambda-Corps | `Lambda-Corps/2025_Robot_ReefScape` | Full Phoenix 6 + commands2 integration |
| FRC 2429 | `aesatchien/FRC2429_2025` | Clean Python project structure |
| HuskieRobotics 3061 | `HuskieRobotics/3061-lib` | Reusable swerve library for SDS modules |
| Aurobots 7456 | `Aurobots7456/SwerveDrive` | Python swerve drive implementation |
| Raptacon | `Raptacon/Robot-2025` | Modern 2025 seasonal code |

### Key Resources
- [RobotPy Docs](https://robotpy.readthedocs.io/)
- [Phoenix 6 Python Examples](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/python)
- [WPILib Command-Based (Python)](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [PathPlannerLib Docs](https://pathplanner.dev/)
- [AdvantageScope](https://docs.advantagescope.org/)
- [pyfrc Testing](https://robotpy.readthedocs.io/en/stable/testing.html)
