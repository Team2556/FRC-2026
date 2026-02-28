# System Test Program

## Purpose
This system test program is designed for pit testing (maintenance area at competitions) to verify all robot subsystems are functioning correctly.

## Prerequisites
Before running the system test, ensure:
- **Robot is on a stand** - wheels should not be touching the ground
- **Intake has room to deploy** - ensure no obstructions around the intake mechanism
- **Shooter/hopper is empty** - no game pieces should be loaded

## How to Run the Test

### Using Driver Station
1. Connect to the robot via Driver Station
2. Switch to **Test Mode** (not Disabled, Teleop, or Autonomous)
3. The test will automatically start and run through all subsystems
4. Monitor progress on the SmartDashboard

### Monitoring the Test
The test publishes the following information to SmartDashboard:

- **System Test Status** - Overall test status message
- **System Test Stage** - Current subsystem being tested
- **System Test Passed** - Boolean indicating if test passed
- **System Test Failure** - Error message if test failed

Additionally, each motor's RPM is displayed during testing:
- Intake Motor RPM
- Shooter RPM
- Spindex RPM
- Transfer 1 RPM
- Transfer 2 RPM

## Test Sequence

The test runs through the following stages:

1. **Intake Motor Test** (2 seconds)
   - Spins the intake motor
   - Verifies RPM reaches expected value
   - Checks for stalls (possible obstruction)

2. **Shooter Motor Test** (2 seconds)
   - Spins the shooter motor
   - Verifies RPM reaches expected value
   - Checks for stalls (possible ball in shooter)

3. **Spindex Motor Test** (2 seconds)
   - Spins the spindex/indexer motor
   - Verifies RPM reaches expected value
   - Checks for stalls (possible ball in hopper)

4. **Transfer Motor 1 Test** (2 seconds)
   - Spins transfer motor 1
   - Verifies RPM reaches expected value
   - Checks for stalls

5. **Transfer Motor 2 Test** (2 seconds)
   - Spins transfer motor 2
   - Verifies RPM reaches expected value
   - Checks for stalls

6. **Vision System Test** (1 second)
   - Verifies vision system is responding
   - Checks for valid telemetry data

## Safety Features

The test includes several safety checks:

- **Motor Stall Detection** - If any motor fails to spin up (RPM < 100) after 0.5 seconds, the test stops and fails
- **RPM Verification** - Motors must reach at least 20% of their target RPM to pass
- **Automatic Stop** - All motors are stopped when the test completes or fails
- **Obstruction Detection** - Low RPM indicates possible obstruction (ball in hopper, blocked intake, etc.)

## Test Results

### Passed Test
If all subsystems pass:
- SmartDashboard shows "System Test Passed: True"
- Status shows "PASSED - All systems nominal"

### Failed Test
If any subsystem fails:
- Test immediately stops
- SmartDashboard shows "System Test Passed: False"
- Status shows "FAILED: [reason]"
- Failure message provides specific details

Example failure messages:
- "Intake motor stalled - possible obstruction"
- "Shooter motor below target RPM (got 500, expected ~2500)"
- "Spindex motor stalled - possible ball in hopper"
- "Vision system not responding"

## Troubleshooting

### Motor not spinning
- Check CAN bus connections
- Verify motor controller is powered
- Check for mechanical binding

### Motor stalling immediately
- Remove any game pieces from the mechanism
- Check for physical obstructions
- Verify mechanism can move freely

### RPM below target
- Check motor configuration
- Verify PID tuning is correct
- Look for excessive friction or binding

### Vision system not responding
- Check Limelight power and network connection
- Verify camera is functioning
- Check NetworkTables connection

## Code Location
- System test command: `commands/system_test.py`
- Test integration: `robot.py` (testInit method)
- Command factory: `robotcontainer.py` (getSystemTestCommand method)
