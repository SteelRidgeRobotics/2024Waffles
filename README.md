# Waffles
Waffles is Steel Ridge's second iteration of swerve drive.

## What is swerve drive?
Swerve drive is an omnidirection drive train where all wheels are independently steered and driven.
This gives us a multitude of advantages over a simple tank drive, listed below:

- More accurate positioning when lining up to score.
- Independent rotating of each wheel, which allows us to spin and move separately, allowing for a greater movement freedom.
- More traction with the ground than a mechanum drive train. (Can't be pushed around as easily)
- Flexing on other teams.

# Technical Overview
Our swerve drive is comprised of the following:
- 4 swerve modules in each corner of the base frame
  - 2 motors in each modules (8 motors total)
    - 1 for steering the wheel (the "direction motor")
    - 1 for driving/spinning the wheel (the "drive motor")

We use [Falcon 500](https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/)'s for each motor, 
as well as 1 [CANcoder](https://store.ctr-electronics.com/cancoder/) for each module. (Used for accurately lining up the wheel on startup)

## Software Overview
Inside the robot, each Falcon motor is controlled using a TalonFX motor controller, using the 2024 Phoenix 6 improvements.

The drive motor uses the Pro-only Velocity Torque-Current + FOC to finely control the wheel's acceleration. 
This eliminates a multitude of factors that can change how each wheel performs under different circumstances, which can cause reduced reliability and control during a match.

The direction motor uses MotionMagic to ensure a clean, fluid motion from position A to position B. 
This helps reduce uncertainties during the competition and reduces drift during matches.
