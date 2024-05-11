# Waffles
Waffles is Steel Ridge's second iteration of swerve drive.

## What is swerve drive?
Swerve drive is an omnidirection drive train where all wheels are independently steered and driven.
This gives us a multitude of advantages over a simple tank drive, listed below:

- More accurate positioning when lining up to score.
- Independent rotating of each wheel, which allows us to spin and move separately, allowing for greater movement freedom.
- More traction with the ground than a mechanum drive train. (Can't be pushed around as easily)
- Flexing on other teams.

# General Overview
Waffles contains the following:
- Fused CANcoders for precise steering
- Continuous wrap for optimized steering
- Motion Magic + torque control to increase traction
- Buttons to switching between robot-centric and field-relative drive
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

## Software Overview
Inside the robot, each Falcon motor is controlled using a TalonFX motor controller, using Phoenix 6 Pro improvement.

The TalonFX responsible for controlling the wheel's velocity uses Motion Magic with torque control, allowing precise acceleration to prevent "wheel slip" when accelerating.

The TalonFX responible for rotating the wheel uses Motion Magic with voltage control. We also fuse the module's CANcoder with the Talon's sensor, allowing us to compesnate for play in the gear system.
