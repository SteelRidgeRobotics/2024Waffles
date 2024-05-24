from commands2 import Command, Subsystem

from limelight import LimelightHelpers

import navx

from pathplannerlib.auto import AutoBuilder, HolonomicPathFollowerConfig, ReplanningConfig
from pathplannerlib.commands import PathfindThenFollowPathHolonomic
from pathplannerlib.config import HolonomicPathFollowerConfig
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.path import PathConstraints, PathPlannerPath
from pathplannerlib.controller import PIDConstants

from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState

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
    odometry = SwerveDrive4PoseEstimator(
        kinematics, # The wheel locations on the robot
        navx.getRotation2d(), # The current angle of the robot
        ( # The current recorded positions of all modules
            left_front.get_position(),
            left_rear.get_position(),
            right_front.get_position(),
            right_rear.get_position()
        ),
        Pose2d()
    )
    
    # Simulated navX angle
    gyro_sim = 0
    
    # Widgets (Gyro widget is done in periodic)
    field = Field2d()
    
    Shuffleboard.getTab("Main").add("Field", field).withWidget(BuiltInWidgets.kField)

    # Tell the limelight what AprilTags we want to read (all of them)
    """
    LimelightHelpers.set_fiducial_id_filters_override(
        Constants.Limelight.k_limelight_name, 
        [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    )
    """
    
    def __init__(self) -> None:
        
        # Send Reset Yaw command to Shuffleboard
        Shuffleboard.getTab("Main").add(
            "Reset Yaw",
            self.runOnce(self.reset_yaw) # Simple InstantCommand, nothing crazy
        ).withWidget(BuiltInWidgets.kCommand)
        
        # Configure PathPlanner
        AutoBuilder.configureHolonomic(
            self.odometry.getEstimatedPosition,
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

        # Shows the target pose in the field widget
        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self.field.getObject("target").setPose(pose))

        # Shows the active path on the field widget
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self.field.getObject("active_path").setPoses(poses))
        
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

        ## Vision Odometry ##

        add_vision_estimate = Constants.Limelight.k_enable_vision_odometry
        if not Constants.Limelight.k_use_mega_tag_2: # Mega Tag 1
            
            mega_tag1 = LimelightHelpers.get_botpose_estimate_wpiblue(Constants.Limelight.k_limelight_name)

            # Check if we're confident on where we are on the field
            if mega_tag1.tag_count == 1 and len(mega_tag1.raw_fiducials) == 1:

                if mega_tag1.raw_fiducials[0].ambiguity > .7 \
                    or mega_tag1.raw_fiducials[0].dist_to_camera > 3: # Don't trust mega tag 1 if we're not close to the april tags

                    add_vision_estimate = False 
            
            elif mega_tag1.tag_count == 0:
                add_vision_estimate = False # Obviously, don't add vision measurements if it doesn't see any apriltags

            # Add Vision Measurement
            if add_vision_estimate:
                self.odometry.setVisionMeasurementStdDevs(Constants.Limelight.k_standard_deviations)
                self.odometry.addVisionMeasurement(
                    mega_tag1.pose,
                    mega_tag1.timestamp_seconds
                )
        
        else: # Mega Tag 2

            # Set Robot Orientation
            LimelightHelpers.set_robot_orientation(
                Constants.Limelight.k_limelight_name,
                self.navx.getRotation2d().degrees(),
                0, # Potentially add -self.navx.getRate() here?
                0,
                0,
                0,
                0
            )

            mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(Constants.Limelight.k_limelight_name)

            # If we're spinning or we don't see an apriltag, don't add vision measurements
            if abs(self.navx.getRate()) > 720 or  \
                mega_tag2.tag_count == 0:

                add_vision_estimate = False

            # Add Vision Measurement
            if add_vision_estimate:
                self.odometry.setVisionMeasurementStdDevs(Constants.Limelight.k_standard_deviations)
                self.odometry.addVisionMeasurement(
                    mega_tag2.pose,
                    mega_tag2.timestamp_seconds
                )

            
        # Update the field pose
        self.field.setRobotPose(self.odometry.getEstimatedPosition())

        ## Show swerve module direction on robot
        robot_pose = self.field.getRobotPose()

        # Left Front (We subtract the size of the wheel pose on the field widget to make the wheel appear to be inside the robot's frame.)
        left_front_pose = robot_pose.transformBy(
            Transform2d(
                    Constants.Drivetrain.ModuleLocations.k_left_front_location - Translation2d(Constants.Drivetrain.ModuleLocations.k_wheel_size, Constants.Drivetrain.ModuleLocations.k_wheel_size), 
                    self.left_front.get_angle()
            )
        )
        
        # Left Rear
        left_rear_pose = robot_pose.transformBy(
            Transform2d(
                    Constants.Drivetrain.ModuleLocations.k_left_rear_location - Translation2d(-Constants.Drivetrain.ModuleLocations.k_wheel_size, Constants.Drivetrain.ModuleLocations.k_wheel_size), 
                    self.left_rear.get_angle()
            )
        )

        # Right Front
        right_front_pose = robot_pose.transformBy(
            Transform2d(
                    Constants.Drivetrain.ModuleLocations.k_right_front_location - Translation2d(Constants.Drivetrain.ModuleLocations.k_wheel_size, -Constants.Drivetrain.ModuleLocations.k_wheel_size), 
                    self.right_front.get_angle()
            )
        )

        # Right Rear
        right_rear_pose = robot_pose.transformBy(
            Transform2d(
                    Constants.Drivetrain.ModuleLocations.k_right_rear_location - Translation2d(-Constants.Drivetrain.ModuleLocations.k_wheel_size, -Constants.Drivetrain.ModuleLocations.k_wheel_size), 
                    self.right_rear.get_angle()
            )
        )

        # Send all the module poses to the field widget
        self.field.getObject("modules").setPoses(
            [left_front_pose, left_rear_pose, right_front_pose, right_rear_pose]
        )
        
        
        # Update Gyro widget (shuffleboard REALLY likes to duplicate widgets, very annoying)
        # I'll just use SmartDashboard for this instead: Elastic won't auto-populate, so no worries about incorrect tabs or anything
        SmartDashboard.putNumber("Yaw", -self.get_yaw().degrees())
        
    def get_yaw(self) -> Rotation2d:
        """Gets the rotation of the robot."""
        
        # If we're in simulation, then calculate how much the robot *should* be moving.
        # Otherwise, just read the current NavX heading.
        if RobotBase.isReal():
            angle = self.navx.getRotation2d()
        else:
            angle = Rotation2d.fromDegrees(self.gyro_sim)
            
        return angle
    
    def reset_yaw(self) -> None:
        """Resets the navX's angle to 0. Also resets the simulation angle as well."""
        
        # Do I really need to explain this in-depth?
        self.gyro_sim = 0
        self.navx.reset()
    
    def simulate_gyro(self, degrees_per_second: float) -> None:
        """Calculates the current angle of the gyro from the degrees per second traveled."""
        
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

    def pathfind_to_pose(self, end_pose: Pose2d) -> Command:
        """Uses Pathplanner's on-the-fly path generation to drive to a given point on the field."""

        pathfind_command = AutoBuilder.pathfindToPose(
            end_pose, 
            PathConstraints(
                Constants.Drivetrain.k_max_attainable_speed, 
                rot_to_meters(12),
                Constants.Drivetrain.k_max_rot_rate,
                Constants.Drivetrain.k_max_rot_rate
            ),
        )

        return pathfind_command
    
    def pathfind_to_path(self, path_name: str, path_constraints: PathConstraints = PathConstraints(3.5, 3.5, 7.85398, 7.85398), rotation_delay_distance = 0) -> Command:
        """Uses Pathplanner's on-the-fly path generation to drive to and executes the given PathPlannerPath"""

        path = PathPlannerPath.fromPathFile(path_name)

        return PathfindThenFollowPathHolonomic(
            path, # The path to pathfind to
            path_constraints, # Constraints while pathfinding (max velocity, max rotation, etc)
            self.odometry.getEstimatedPosition, # Pose supplier
            self.get_robot_speed, # Speed Supplier
            self.drive_robot_centric, # Speed Consumer
            HolonomicPathFollowerConfig( # Holonomic-specific config (same as constructor, copied down here so we can invert paths when on the red alliance)
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
                Constants.Drivetrain.k_max_attainable_speed, # Max module speed
                Constants.Drivetrain.k_drive_base_radius, # Distance from center of the robot to a swerve module
                ReplanningConfig() # Replanning Config
            ),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # "hey caden when do we invert the path"
            self, # We ARE a DRIVE-TEAM!!!!!!
            rotation_delay_distance # Amount the robot needs to move before it rotates (when pathfinding)
        )
