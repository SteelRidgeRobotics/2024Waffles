# Waffles
Waffles is Steel Ridge Robotics' (FRC 6343) official swerve drive codebase, which began development in late 2022.

## What is swerve drive?
Swerve drive is an omnidirectional drive train where all wheels are independently steered and driven.
This gives us unique advantages over other drive trains:

- Strafing, driving left to right, is possible and more efficient than mechanum drive trains.
- More speed conserved when turning, since the robot has no need to rotate.
- More traction with the ground than a mechanum drive train. (Can't be pushed around as easily)
- Flexing on other teams.

# Software Overview
Waffles contains the following:
- Fused CANcoders for precise steering
- Continuous wrap for optimized steering
- Motion Magic combined with [field-orientated control](https://en.m.wikipedia.org/wiki/Vector_control_(motor)) to reduce skidding and improve handling.
- Ability to switch between robot-centric (forward is the direction the robot faces) and field-relative (forward is across the field from the driver) modes.
- Skidding and collision detection, to compensate for errors in autonomous driving.
- Advanced joystick mapping to ensure maximum speed.
- PathPlanner support (includes Choreo configuration)
- NavX simulation
- Full simulation support for all modes
- Auto Trajectories Visualized in Shuffleboard
- Shuffleboard and Elastic Support

# Technical Overview
Our swerve drive is comprised of the following:
- 4 swerve modules in each corner of the base frame
  - 2 motors in each modules (8 motors total)
    - 1 for steering the wheel (the "steer motor")
    - 1 for driving/spinning the wheel (the "drive motor")

We use [Falcon 500](https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/)s for each motor, 
as well as 1 [CANcoder](https://store.ctr-electronics.com/cancoder/) for each module.