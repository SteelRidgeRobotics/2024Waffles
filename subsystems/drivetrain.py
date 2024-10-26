from commands2 import Command, Subsystem

from limelight import LimelightHelpers

import navx

from ntcore import NetworkTableInstance

from pathplannerlib.auto import AutoBuilder, HolonomicPathFollowerConfig, ReplanningConfig
from pathplannerlib.commands import PathfindThenFollowPathHolonomic
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import HolonomicPathFollowerConfig
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.path import PathConstraints, PathPlannerPath
from pathplannerlib.controller import PIDConstants

from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard, Timer
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.trajectory import TrapezoidProfile
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.controller import ProfiledPIDController
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState

from constants import Constants
from conversions import *
from subsystems.module import SwerveModule


class Drivetrain(Subsystem):
    """Swerve drivetrain :partying_face:"""
    
    # Creating all modules (in a tuple for organization)
    modules = (
        SwerveModule(
            Constants.CanIDs.k_left_front_drive,
            Constants.CanIDs.k_left_front_direction,
            Constants.CanIDs.k_left_front_encoder,
            Constants.CanOffsets.k_left_front_offset
        ),
        SwerveModule(
            Constants.CanIDs.k_left_rear_drive,
            Constants.CanIDs.k_left_rear_direction,
            Constants.CanIDs.k_left_rear_encoder,
            Constants.CanOffsets.k_left_rear_offset
        ),
        SwerveModule(
            Constants.CanIDs.k_right_front_drive,
            Constants.CanIDs.k_right_front_direction,
            Constants.CanIDs.k_right_front_encoder,
            Constants.CanOffsets.k_right_front_offset
        ),
        SwerveModule(
            Constants.CanIDs.k_right_rear_drive,
            Constants.CanIDs.k_right_rear_direction,
            Constants.CanIDs.k_right_rear_encoder,
            Constants.CanOffsets.k_right_rear_offset
        )
    )

    # Kinematics
    kinematics = SwerveDrive4Kinematics(
        Constants.Drivetrain.k_module_locations[0], 
        Constants.Drivetrain.k_module_locations[1],
        Constants.Drivetrain.k_module_locations[2], 
        Constants.Drivetrain.k_module_locations[3]
    )
    
    # NavX Setup
    gyro = navx.AHRS.create_spi()
    gyro.reset()

    gyro_sim = 0 # Simulated angle
    
    # Field Widget
    field = Field2d()
    Shuffleboard.getTab("Main").add("Field", field).withWidget(BuiltInWidgets.kField)

    # PathPlanner Config
    path_follower_config = HolonomicPathFollowerConfig(
        # Holonomic-specific config
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
    )

    ## NetworkTable Publishing (for logging)
    drivetrain_nt = NetworkTableInstance.getDefault().getTable("Drivetrain")

    # Odometry
    robot_pose_publisher = drivetrain_nt.getStructTopic("RobotPose", Pose2d).publish()
    latency_compensated_pose_publisher = drivetrain_nt.getStructTopic("LatencyCompPose", Pose2d).publish()
    target_pose_publisher = drivetrain_nt.getStructTopic("PPTarget", Pose2d).publish()
    vision_pose_publisher = drivetrain_nt.getStructTopic("VisionPose", Pose2d).publish()

    # Swerve
    module_state_publisher = drivetrain_nt.getStructArrayTopic("ModuleStates", SwerveModuleState).publish()
    module_target_publisher = drivetrain_nt.getStructArrayTopic("ModuleTargets", SwerveModuleState).publish()
    skid_ratio_publisher = drivetrain_nt.getFloatTopic("SkidRatio").publish()

    # Turn PID for Fully Field-Relative driving
    turn_PID = ProfiledPIDController(
        Constants.Drivetrain.k_turn_p, 
        0,
        Constants.Drivetrain.k_turn_d,
        TrapezoidProfile.Constraints(Constants.Drivetrain.k_max_rot_rate, 30)
    )
    turn_PID.enableContinuousInput(-math.pi, math.pi)

    @staticmethod
    def get_module_positions() -> tuple[SwerveModulePosition]:
        """Returns all reported SwerveModulePositions for every module."""

        positions = []
        for module in Drivetrain.modules:
            positions.append(module.get_position())
        return tuple(positions)
    
    @staticmethod
    def get_module_states() -> tuple[SwerveModuleState]:
        """Returns all reported SwerveModuleStates for every module."""

        states = []
        for module in Drivetrain.modules:
            states.append(module.get_state())
        return tuple(states)
    
    @staticmethod
    def get_module_targets() -> tuple[SwerveModuleState]:
        """Returns all module target states."""

        targets = []
        for module in Drivetrain.modules:
            targets.append(module.get_target())
        return targets
    
    @staticmethod
    def get_module_angles() -> tuple[Rotation2d]:
        """Returns the angle for every module as a tuple of Rotation2d's."""

        angles = []
        for module in Drivetrain.modules:
            angles.append(module.get_angle())
        return tuple(angles)

    def __init__(self, starting_pose: Pose2d = Pose2d()) -> None:

        # Odometry
        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics, # The wheel locations on the robot
            self.gyro.getRotation2d(), # The current angle of the robot
            Drivetrain.get_module_positions(),
            starting_pose
        )
        
        # Send Reset Yaw command to Shuffleboard
        Shuffleboard.getTab("Main").add(
            "Reset Yaw",
            self.runOnce(self.reset_yaw) # Simple InstantCommand, nothing crazy
        ).withWidget(BuiltInWidgets.kCommand)
        
        # Configure PathPlanner
        AutoBuilder.configureHolonomic(
            self.get_latency_compensated_position,
            lambda pose: self.reset_pose(pose),
            self.get_robot_speed,
            lambda speeds: self.drive_robot_centric(speeds),
            self.path_follower_config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # "Hey Caden, when do we flip the path?"
            self # "Yes, this is the drivetrain. Why would I configure an AutoBuilder for an intake, pathplannerlib?"
        )

        PPHolonomicDriveController.setRotationTargetOverride(self.get_rotation_override)

        # Logs the target pose
        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self.target_pose_publisher.set(pose))

        # Shows the active path on the field widget
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self.field.getObject("active_path").setPoses(poses))

        self.prev_target_angle = Rotation2d()

        self.last_odometry_update = 0

    def update_odometry(self) -> None:
        """Reads module positions to update odometry (wow)."""

        timestamp = Timer.getFPGATimestamp()

        self.odometry.updateWithTime(
            timestamp,
            self.get_yaw(), 
            Drivetrain.get_module_positions()
        )

        # Log Everything
        self.module_target_publisher.set(list(Drivetrain.get_module_targets()))
        self.module_state_publisher.set(list(Drivetrain.get_module_states()))

        self.robot_pose_publisher.set(self.odometry.getEstimatedPosition())
        self.latency_compensated_pose_publisher.set(self.get_latency_compensated_position())

        self.last_odometry_update = timestamp

    def get_odometry_latency(self) -> float:
        """Returns the time since the last odometry update, in seconds."""
        return Timer.getFPGATimestamp() - self.last_odometry_update

    def get_latency_compensated_position(self) -> Pose2d:
        """Interpolates the current pose of the robot by transforming the estimated position by the velocity."""
        estimated_position = self.odometry.getEstimatedPosition()

        estimated_velocity = self.get_robot_speed()

        latency = self.get_odometry_latency()

        velocity_transform = Transform2d(estimated_velocity.vx*latency, estimated_velocity.vy*latency, Rotation2d(estimated_velocity.omega*latency))

        return estimated_position.transformBy(velocity_transform)

    def update_vision_estimates(self) -> None:
        """Uses Limelight MegaTag to help prevent pose drift."""

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

                self.vision_pose_publisher.set(mega_tag1.pose)
        
        else: # Mega Tag 2

            # Set Robot Orientation
            LimelightHelpers.set_robot_orientation(
                Constants.Limelight.k_limelight_name,
                self.gyro.getRotation2d().degrees(),
                0, # Potentially add -self.navx.getRate() here LATER, according to Chief Delphi it's untested
                0, 0, 0, 0
            )

            mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(Constants.Limelight.k_limelight_name)

            # If we're spinning or we don't see an apriltag, don't add vision measurements
            if abs(self.gyro.getRate()) > 720 or mega_tag2.tag_count == 0:
                add_vision_estimate = False

            # Add Vision Measurement
            if add_vision_estimate:
                self.odometry.setVisionMeasurementStdDevs(Constants.Limelight.k_standard_deviations)
                self.odometry.addVisionMeasurement(
                    mega_tag2.pose,
                    mega_tag2.timestamp_seconds
                )

                self.vision_pose_publisher.set(mega_tag2.pose)

    def periodic(self) -> None:

        estimated_position = self.get_latency_compensated_position()

        # Update the field pose
        self.field.setRobotPose(estimated_position)

        # Log everything
        self.skid_ratio_publisher.set(self.get_skidding_ratio())

        ## Show swerve modules on robot
        if not DriverStation.isFMSAttached() and not RobotBase.isReal():
            module_angles = Drivetrain.get_module_angles()

            module_poses = []
            for i in range(len(self.modules)):

                translation = Constants.Drivetrain.k_module_locations[i]

                sim_offset = Translation2d(
                    math.copysign(Constants.Drivetrain.k_sim_wheel_size[0], translation.X()), 
                    math.copysign(Constants.Drivetrain.k_sim_wheel_size[1], translation.Y())
                )

                module_poses.append(
                    self.field.getRobotPose().transformBy(
                        Transform2d(
                            translation - sim_offset,
                            module_angles[i]
                        )
                    )
                )
            self.field.getObject("modules").setPoses(module_poses)
        elif len(self.field.getObject("modules").getPoses()) > 1:
            self.field.getObject("modules").setPose(-1000, -1000, Rotation2d())

        # Update Gyro widget
        SmartDashboard.putNumber("Yaw", -self.get_yaw().degrees())

        # Update Skid Ratio
        SmartDashboard.putNumber("Skidding Ratio", Drivetrain.get_skidding_ratio())

    @staticmethod
    def get_skidding_ratio() -> float:
        """
        Translated from https://www.chiefdelphi.com/t/has-anyone-successfully-implemented-orbits-odometry-skid-detection/468257/3
        Built from concepts mentioned in 1690's Software Session, https://youtu.be/N6ogT5DjGOk?feature=shared&t=1674

        Returns the skidding ratio to determine how much the chassis is skidding.
        The skidding ratio is the ratio between the maximum and minimum magnitude of the translational speed of the modules.
        """

        module_states = Drivetrain.get_module_states()

        measured_angular_velocity = Drivetrain.kinematics.toChassisSpeeds(module_states).omega

        state_rotation = Drivetrain.kinematics.toSwerveModuleStates(ChassisSpeeds(0, 0, measured_angular_velocity))

        module_translation_magnitudes = []

        for i in range(len(module_states)):
            module_state_vector = Drivetrain._module_state_to_velocity_vector(module_states[i])

            module_rotation_vector = Drivetrain._module_state_to_velocity_vector(state_rotation[i])
            module_translation_vector = module_state_vector - module_rotation_vector

            module_translation_magnitudes.append(module_translation_vector.norm())

        max_translation = 0
        min_translation = math.inf
        for mag in module_translation_magnitudes:
            max_translation = max(max_translation, mag)
            min_translation = min(min_translation, mag)
        
        try:
            return max_translation / min_translation
        except ZeroDivisionError:
            return 1.0

    @staticmethod
    def _module_state_to_velocity_vector(module_state: SwerveModuleState) -> Translation2d:
        return Translation2d(module_state.speed, module_state.angle)
        
    def get_yaw(self) -> Rotation2d:
        """Gets the rotation of the robot."""
        
        # If we're in simulation, then calculate how much the robot *should* be moving.
        # Otherwise, just read the current NavX heading.
        if RobotBase.isReal():
            angle = self.gyro.getRotation2d()
        else:
            angle = Rotation2d.fromDegrees(self.gyro_sim)
            
        return angle
    
    def reset_yaw(self) -> None:
        """Resets the navX's angle to 0. Also resets the simulation angle as well."""
        
        self.gyro_sim = 0
        self.gyro.reset()
    
    def simulate_gyro(self, degrees_per_second: float) -> None:
        """Calculates the current angle of the gyro from the degrees per second traveled."""
        
        self.gyro_sim += degrees_per_second * 0.02
        
    def reset_pose(self, pose: Pose2d) -> None:
        """Resets the robot's recorded position on the field via odometry."""
        
        self.odometry.resetPosition(
            self.get_yaw(),
            Drivetrain.get_module_positions(),
            pose
        )
        
    def get_robot_speed(self) -> ChassisSpeeds:
        """Returns the robots current speed (non-field relative)"""
        
        return self.kinematics.toChassisSpeeds(Drivetrain.get_module_states())
        
    def drive_robot_centric(self, speeds: ChassisSpeeds, center_of_rotation: Translation2d = Translation2d(0, 0)) -> None:
        """Drives the robot at the given speeds (from the perspective of the robot)."""
        
        speeds = ChassisSpeeds.discretize(speeds, 0.02) # Make spinning and moving drift less
        
        # Calculate each module's state
        module_speeds = self.kinematics.toSwerveModuleStates(speeds, center_of_rotation)
        
        self.set_desired_module_states(module_speeds)

        # Since PathPlanner pathfinding uses robot-centric, we need to 
        self.prev_target_angle = self.get_yaw()
        self.turn_PID.reset(self.prev_target_angle.radians())
        
        # Set the navX to what the angle should be in simulation
        self.simulate_gyro(speeds.omega_dps)
            
    def drive_field_relative(self, speeds: ChassisSpeeds, center_of_rotation: Translation2d = Translation2d(0, 0)) -> None:
        """Drives the robot at the given speeds (from the perspective of the driver/field)"""

        speeds = ChassisSpeeds.discretize(speeds, 0.02)

        if speeds.omega == 0: # If we don't want to rotate, use the rotation PID.
            rotational_speed = Drivetrain.turn_PID.calculate(self.get_yaw().radians(), self.prev_target_angle.radians())
            speeds = ChassisSpeeds(speeds.vx, speeds.vy, rotational_speed)
        else:
            self.prev_target_angle = self.get_yaw()
            self.turn_PID.reset(self.prev_target_angle.radians())
        
        # Convert to robot-centric
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, self.get_yaw())
        
        # Calculate each module's state
        module_speeds = self.kinematics.toSwerveModuleStates(speeds, center_of_rotation)
        
        self.set_desired_module_states(module_speeds)
        
        # Set the navX to what the angle should be in simulation
        self.simulate_gyro(speeds.omega_dps)

    def drive_fully_field_relative(self, speeds: ChassisSpeeds, target_angle: Rotation2d | None) -> None:
        """Drives the robot at the given speeds (from the perspective of the driver/field) and rotates to the target angle.\n
        If the ChassisSpeeds rotational velocity does not equal 0, we ignore the target angle and instead rotate at the desired velocity."""

        if speeds.omega != 0: # If speeds has a rotational velocity, that should override the desired direction.
            self.drive_field_relative(speeds)
            return

        if target_angle is None:
            target_angle = self.prev_target_angle

        rotational_speed = Drivetrain.turn_PID.calculate(self.get_yaw().radians(), target_angle.radians())

        field_speeds = ChassisSpeeds(speeds.vx, speeds.vy, rotational_speed)
        field_speeds = ChassisSpeeds.discretize(field_speeds, 0.02)

        robo_centric = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vx, speeds.vy, rotational_speed, self.get_yaw())

        self.set_desired_module_states(self.kinematics.toSwerveModuleStates(robo_centric))

        self.prev_target_angle = target_angle

        self.simulate_gyro(robo_centric.omega_dps)
        
    def set_desired_module_states(self, states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        """Sends the given module states to each module."""
        
        # Make sure we aren't traveling at unrealistic speeds
        states = self.kinematics.desaturateWheelSpeeds(
            states, Constants.Drivetrain.k_max_attainable_speed
        )
        
        # Set each state to the correct module
        for i, module in enumerate(self.modules):

            if states[i].speed == 0:
                module.stop()
            else:
                module.set_desired_state(states[i])

    def pathfind_to_pose(self, end_pose: Pose2d) -> Command:
        """Uses Pathplanner's on-the-fly path generation to drive to a given point on the field."""

        return AutoBuilder.pathfindToPose(
            end_pose, 
            PathConstraints(
                Constants.Drivetrain.k_max_attainable_speed, 
                rot_to_meters(12),
                Constants.Drivetrain.k_max_rot_rate,
                Constants.Drivetrain.k_max_rot_rate
            ),
        )
    
    def pathfind_to_path(self, path_name: str, path_constraints: PathConstraints = PathConstraints(3.5, 3.5, 7.85398, 7.85398), rotation_delay_distance = 0) -> Command:
        """Uses Pathplanner's on-the-fly path generation to drive to and executes the given PathPlannerPath"""

        path = PathPlannerPath.fromPathFile(path_name)

        return PathfindThenFollowPathHolonomic(
            path, # The path to pathfind to
            path_constraints, # Constraints while pathfinding (max velocity, max rotation, etc)
            self.odometry.getEstimatedPosition, # Pose supplier
            self.get_robot_speed, # Speed Supplier
            self.drive_robot_centric, # Speed Consumer
            self.path_follower_config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # "hey caden when do we invert the path"
            self, # We ARE a DRIVE-TEAM!!!!!!
            rotation_delay_distance # Amount the robot needs to move before it rotates (when pathfinding)
        )
    
    def get_rotation_override(self) -> Rotation2d:
        """Rotation override for PathPlanner.
        See https://pathplanner.dev/pplib-override-target-rotation.html"""

        # Look at note if we're in autonomous
        if False:
            
            # Look at center of field (broken, but a good example)
            translation = Transform2d(self.odometry.getEstimatedPosition(), Pose2d(8.321, 4.092, self.odometry.getEstimatedPosition().rotation())).translation()
            x, y = translation.X(), translation.Y()

            return Rotation2d(math.atan2(y, x))
        
        else:
            return None
