# FRC 2026 Rebuilt
*Team 2556 Radioactive Roaches*

---

## Button Bindings

### Controller 1
- **Left Joystick** - Move (field-centric)
- **Right Joystick** - Rotate
- **Right Bumper** - Auto drive toward fuel (Ben's magic button)
- **Right Trigger** - Shoot + align to best spot
- **Left Bumper** - Magic button
- **Left Trigger** - Move slower
- **Letter Buttons** - Specific paths

### Controller 2
- **Right Trigger** *(hold)* - Toggle intake
- **POV Up** *(press)* - Climb up

- **POV Down** *(press)* - Climb down
- **B** *(hold)* - Spin spindexer / transfer
- **Y** *(hold)* - Spin shooter

#### Controller 2 Ideas
- Manually move the hood
- Deploy but unintake button (jam recovery)
- **Left Joystick** - Manually adjust offset angle for hub shooting

---

## CAN IDs
> For the authoritative reference, see `constants/canbus.py`

### RoboRIO
- **RoboRIO** - 0

### Drive (1-13)
- **FL Drive** - 1
- **FR Drive** - 2
- **BL Drive** - 3
- **BR Drive** - 4
- **FL Steer** - 5
- **FR Steer** - 6
- **BL Steer** - 7
- **BR Steer** - 8
- **FL CANCoder** - 9
- **FR CANCoder** - 10
- **BL CANCoder** - 11
- **BR CANCoder** - 12
- **Pigeon** - 13

### Intake (16-20)
- **Spinny Intake Motor** - 16
- **Left Intake Deploy** - 17
- **Right Intake Deploy** - 18

### Transfer (21-25)
- **Spindexer** - 21
- **Up Transfer** - 22

### Shooter (26-30)
- **Shooter Bottom** - 26
- **Shooter Top** - 27
- **Hood Motor** - 28

### Climb (41-45)
- **Motor** - 41

### Everything Else (31+)
- **REV Power Hub** - 31
- **CANDle** - 32

---

## Checklist

### Things to do first
- Make Robot Connect (Radio, RIO)
- Set CAN IDs
- Make every (non-dual) motor spin correctly with high quality formware

### Drivetrain/Vision
- Limelight config to correct positions
- See if it drives normally

### Intake
- Check for correct logic
- Tune PIDs and other values
- Check if it actually intakes

### Shooting Subsystems
- Check if transfer works
- Check if hood works
- Check if increment works
- Tune everything to okay values (actually tune later)
- Get data points for linear interpolation (with hood angles)

### ConditionalAlignAndShoot
- Get it to work perfectly with no problems

### All Paths
- Test auto stop shooting when near trench
- Test every path ever

Then a bunch of practice + testing