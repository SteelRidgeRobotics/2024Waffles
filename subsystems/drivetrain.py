from commands2 import Subsystem

import navx

from pathplannerlib.auto import AutoBuilder, HolonomicPathFollowerConfig, PathPlannerAuto, ReplanningConfig
from pathplannerlib.controller import PIDConstants

from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModuleState


from constants import Constants
from conversions import *
from subsystems.module import SwerveModule


class Drivetrain(Subsystem):
    """Swerve drivetrain :partying_face:"""
    
    # Creating all modules
    left_front = SwerveModule(
        Constants.CanIDs.k_left_front_drive,
        Constants.CanIDs.k_left_front_direction,
        Constants.CanIDs.k_left_front_encoder,
        Constants.CanOffsets.k_left_front_offset
    )
    
    left_rear = SwerveModule(
        Constants.CanIDs.k_left_rear_drive,
        Constants.CanIDs.k_left_rear_direction,
        Constants.CanIDs.k_left_rear_encoder,
        Constants.CanOffsets.k_left_rear_offset
    )
    
    right_front = SwerveModule(
        Constants.CanIDs.k_right_front_drive,
        Constants.CanIDs.k_right_front_direction,
        Constants.CanIDs.k_right_front_encoder,
        Constants.CanOffsets.k_right_front_offset
    )
    
    right_rear = SwerveModule(
        Constants.CanIDs.k_right_rear_drive,
        Constants.CanIDs.k_right_rear_direction,
        Constants.CanIDs.k_right_rear_encoder,
        Constants.CanOffsets.k_right_rear_offset
    )
    
    # NavX Setup
    navx = navx.AHRS.create_spi()
    navx.reset()
    
    # Kinematics
    kinematics = SwerveDrive4Kinematics(
        Constants.Drivetrain.ModuleLocations.k_left_front_location, 
        Constants.Drivetrain.ModuleLocations.k_left_rear_location,
        Constants.Drivetrain.ModuleLocations.k_right_front_location, 
        Constants.Drivetrain.ModuleLocations.k_right_rear_location
    )
    
    # Odometry
    odometry = SwerveDrive4Odometry(
        kinematics, # The wheel locations on the robot
        navx.getRotation2d(), # The current angle of the robot
        ( # The current recorded positions of all modules
            left_front.get_position(),
            left_rear.get_position(),
            right_front.get_position(),
            right_rear.get_position()
        )
    )
    
    # Simulated navX angle
    gyro_sim = 0
    
    # Widgets (Gyro widget is done in periodic)
    field = Field2d()
    
    Shuffleboard.getTab("Main").add("Field", field).withWidget(BuiltInWidgets.kField)
    
    def __init__(self) -> None:
        
        # Send Reset Yaw command to Shuffleboard
        Shuffleboard.getTab("Main").add(
            "Reset Yaw",
            self.runOnce(self.reset_yaw) # Simple InstantCommand, nothing crazy
        ).withWidget(BuiltInWidgets.kCommand)
        
        # Configure PathPlanner
        AutoBuilder.configureHolonomic(
            self.odometry.getPose,
            lambda pose: self.reset_pose(pose),
            self.get_robot_speed,
            lambda speeds: self.drive_robot_centric(speeds),
            HolonomicPathFollowerConfig( # Holonomic-specific config
                PIDConstants( # PID for translation
                        Constants.PathPlanner.k_translation_p,
                        Constants.PathPlanner.k_translation_i,
                        Constants.PathPlanner.k_translation_d,
                        Constants.PathPlanner.k_translation_i_zone
                    ), 
                PIDConstants( # PID for rotation
                        Constants.PathPlanner.k_rotation_p,
                        Constants.PathPlanner.k_rotation_i,
                        Constants.PathPlanner.k_rotation_d,
                        Constants.PathPlanner.k_rotation_i_zone
                    ), 
                Constants.Drivetrain.k_max_attainable_speed, # Max module speed (matches the one in PathPlanner)
                Constants.Drivetrain.k_drive_base_radius, # Distance from center of the robot to a swerve module
                ReplanningConfig() # Replanning Config (check the docs, this is hard to explain)
            ),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # "Hey Caden, when do we flip the path?"
            self # "Yes, this is the drivetrain. Why would I configure an AutoBuilder for an intake, pathplannerlib?"
        )
        
    def periodic(self) -> None:
        
        # Update the odometry...
        self.odometry.update(
            self.get_yaw(), 
            (
                self.left_front.get_position(),
                self.left_rear.get_position(),
                self.right_front.get_position(),
                self.right_rear.get_position()
            )
        )
        
        # ...then update the field pose
        self.field.setRobotPose(self.odometry.getPose())
        
        # Update Gyro widget (shuffleboard REALLY likes to duplicate widgets, very annoying)
        # I'll just use SmartDashboard for this instead: Elastic won't auto-populate, so no worries about incorrect tabs or anything
        SmartDashboard.putNumber("Yaw", -self.get_yaw().degrees())
        
    def get_yaw(self) -> Rotation2d:
        """Gets the rotation of the robot."""
        
        # If we're in simulation, then calculate how much the robot *should* be moving.
        # Otherwise, just read the current NavX heading.
        if RobotBase.isReal():
            angle = self.nav_x.getRotation2d()
        else:
            angle = Rotation2d.fromDegrees(self.gyro_sim)
            
        return angle
    
    def reset_yaw(self) -> None:
        """Resets the navX's angle to 0. Also resets the simulation angle as well."""
        
        # Do I really need to explain this in-depth?
        self.gyro_sim = 0
        self.navx.reset()
    
    def simulate_gyro(self, degrees_per_second: float) -> None:
        """Calculates the current angle of the gyro from the degrees per second travelled."""
        
        self.gyro_sim += degrees_per_second * 0.02
        
    def reset_pose(self, pose: Pose2d) -> None:
        """Resets the robot's recorded position on the field via odometry."""
        
        self.odometry.resetPosition(
            self.get_yaw(),
            (
                self.left_front.get_position(),
                self.left_rear.get_position(),
                self.right_front.get_position(),
                self.right_rear.get_position()
            ),
            pose
        )
        
    def get_robot_speed(self) -> ChassisSpeeds:
        """Returns the robots current speed (non-field relative)"""
        
        return self.kinematics.toChassisSpeeds(
            (
                self.left_front.get_state(),
                self.left_rear.get_state(),
                self.right_front.get_state(),
                self.right_rear.get_state()
            ) 
        )
        
    def drive_robot_centric(self, speeds: ChassisSpeeds, center_of_rotation: Translation2d = Translation2d(0, 0)) -> None:
        """Drives the robot at the given speeds (from the perspective of the robot)."""
        
        speeds = ChassisSpeeds.discretize(speeds, 0.02) # Make spinning and moving drift less
        
        # Calculate each module's state
        module_speeds = self.kinematics.toSwerveModuleStates(speeds, center_of_rotation)
        
        self.set_desired_module_states(module_speeds)
        
        # Set the navX to what the angle should be in simulation
        self.simulate_gyro(speeds.omega_dps)
            
    def drive_field_relative(self, speeds: ChassisSpeeds, center_of_rotation: Translation2d = Translation2d(0, 0)) -> None:
        """Drives the robot at the given speeds (from the perspective of the driver/field)"""
        
        speeds = ChassisSpeeds.discretize(speeds, 0.02)
        
        # Convert to robot-centric
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, self.get_yaw())
        
        # Calculate each module's state
        module_speeds = self.kinematics.toSwerveModuleStates(speeds, center_of_rotation)
        
        self.set_desired_module_states(module_speeds)
        
        # Set the navX to what the angle should be in simulation
        self.simulate_gyro(speeds.omega_dps)
        
    def set_desired_module_states(self, states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        """Sends the given module states to each module."""
        
        # Make sure we aren't traveling at unrealistic speeds
        left_front_state, left_rear_state, right_front_state, right_rear_state = self.kinematics.desaturateWheelSpeeds(
            states, Constants.Drivetrain.k_max_attainable_speed
        )
        
        # Set each state to the correct module
        self.left_front.set_desired_state(left_front_state)
        self.left_rear.set_desired_state(left_rear_state)
        self.right_front.set_desired_state(right_front_state)
        self.right_rear.set_desired_state(right_rear_state)
        
    def load_auto_trajectory(self, auto: PathPlannerAuto) -> None:
        """Visualizes the loaded auto trajectory onto the Field."""
        
        # Decompile auto into individual paths
        paths = PathPlannerAuto.getPathGroupFromAutoFile(auto.getName())
        
        # Get every pose and combine them all into 1 list
        auto_poses = []
        for path in paths:
            
            # Flip the path if we're on the opposite side of the field
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                path = path.flipPath()
            
            poses = path.getPathPoses()
            for pose in poses:
                auto_poses.append(pose)
                
        # Generate the trajectory by passing in all poses
        self.field.getObject("auto_trajectory").setPoses(auto_poses)
        
    def clear_auto_trajectory(self) -> None:
        """Clears the loaded auto trajectory onto the Field."""
        
        self.field.getObject("auto_trajectory").setPose(Pose2d(-1000, -1000, 0))
    