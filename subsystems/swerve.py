import math

import navx
from commands2 import Command, CommandScheduler, Subsystem
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import (HolonomicPathFollowerConfig, PIDConstants,
                                   ReplanningConfig)
from phoenix6.configs.cancoder_configs import *
from phoenix6.configs.talon_fx_configs import *
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.controls import *
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls.motion_magic_voltage import MotionMagicVoltage
from phoenix6.signals import *
from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (ChassisSpeeds, SwerveDrive4Kinematics,
                                SwerveDrive4Odometry, SwerveModulePosition,
                                SwerveModuleState)

from constants import *


class SwerveModule(Subsystem):
    """
    Takes inputted SwerveModuleStates and moves the direction and drive motor to the selected positions.

    The direction motor rotates the wheel into position.
    The drive motor spins the wheel to move.
    """

    def __init__(self, module_name: str, drive_motor_constants: DriveMotorConstants, direction_motor_constants: DirectionMotorConstants, CANcoder_id: int, CAN_offset: float) -> None:
        super().__init__()

        self.module_name = module_name

        self.turning_encoder = CANcoder(CANcoder_id, "rio")
        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor = MagnetSensorConfigs().with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE).with_magnet_offset(CAN_offset).with_absolute_sensor_range(AbsoluteSensorRangeValue.UNSIGNED_0_TO1)
        self.turning_encoder.configurator.apply(encoder_config)
        
        self.drive_motor = TalonFX(drive_motor_constants.motor_id, "rio")
        drive_motor_constants.apply_configuration(self.drive_motor)

        self.direction_motor = TalonFX(direction_motor_constants.motor_id, "rio")
        direction_motor_constants.apply_configuration(self.direction_motor)

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(rots_to_degs(self.direction_motor.get_position().value))
    
    def reset_sensor_position(self) -> None:
        pos = -self.turning_encoder.get_absolute_position().value
        self.direction_motor.set_position(pos)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(rots_to_meters(self.drive_motor.get_velocity().value), self.get_angle())
    
    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(rots_to_meters(self.drive_motor.get_position().value), self.get_angle())

    def set_desired_state(self, desiredState: SwerveModuleState) -> None:
        desiredState = SwerveModuleState.optimize(desiredState, self.get_angle())
        
        self.drive_motor.set_control(VelocityTorqueCurrentFOC(meters_to_rots(desiredState.speed, k_drive_gear_ratio)))
        self.drive_motor.sim_state.set_rotor_velocity(meters_to_rots(desiredState.speed, k_drive_gear_ratio))
        
        self.direction_motor.set_control(MotionMagicVoltage(degs_to_rots(desiredState.angle.degrees())))
        self.direction_motor.sim_state.set_raw_rotor_position(degs_to_rots(desiredState.angle.degrees()))

class Swerve(Subsystem):
    navx = navx.AHRS.create_spi()

    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1),
                                        Translation2d(1, -1), Translation2d(-1, -1)) # LF, LR, RF, RR
    
    field = Field2d()
    
    left_front: SwerveModule = SwerveModule("LF", DriveMotorConstants(MotorIDs.LEFT_FRONT_DRIVE), DirectionMotorConstants(MotorIDs.LEFT_FRONT_DIRECTION), CANIDs.LEFT_FRONT, -0.473388671875)
    left_rear: SwerveModule = SwerveModule("LR", DriveMotorConstants(MotorIDs.LEFT_REAR_DRIVE), DirectionMotorConstants(MotorIDs.LEFT_REAR_DIRECTION), CANIDs.LEFT_REAR, -0.9990234375)
    right_front: SwerveModule = SwerveModule("RF", DriveMotorConstants(MotorIDs.RIGHT_FRONT_DRIVE), DirectionMotorConstants(MotorIDs.RIGHT_FRONT_DIRECTION), CANIDs.RIGHT_FRONT, -0.39990234375)
    right_rear: SwerveModule = SwerveModule("RR", DriveMotorConstants(MotorIDs.RIGHT_REAR_DRIVE), DirectionMotorConstants(MotorIDs.RIGHT_REAR_DIRECTION), CANIDs.RIGHT_REAR, -0.08056640625)
    
    def __init__(self):
        super().__init__()

        self.odometry = SwerveDrive4Odometry(self.kinematics, self.get_angle(),
                                             (self.left_front.get_position(), self.left_rear.get_position(),
                                              self.right_front.get_position(), self.right_rear.get_position()))

        SmartDashboard.putData(self.field)
        SmartDashboard.putData("Reset Odometry", self.reset_odometry_command())

        self.chassis_speed = ChassisSpeeds()
        self.target_rad = 0
        
        if not AutoBuilder.isConfigured():
            # https://pathplanner.dev/pplib-getting-started.html#holonomic-swerve
            AutoBuilder.configureHolonomic(
                lambda: self.get_pose(),
                lambda pose: self.reset_odometry(pose),
                lambda: self.get_chassis_speeds(),
                lambda chassisSpeed: self.drive(chassisSpeed, field_relative=False),
                HolonomicPathFollowerConfig(
                    PIDConstants(0.0, 0.0, 0.0, 0.0), # translation
                    PIDConstants(0.0, 0.0, 0.0, 0.0), # rotation
                    Waffles.k_max_speed / 4,
                    Waffles.k_drive_base_radius,
                    ReplanningConfig(enableInitialReplanning=False)
                ),
                lambda: self.should_flip_auto_path(),
                self
            )
        
        self.navx.reset()

    def should_flip_auto_path(self) -> bool:
        # Flips the PathPlanner path if we're on the red alliance
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def run_auto(self, auto: PathPlannerAuto) -> None:
        self.runOnce(lambda: auto)

    def get_angle(self) -> Rotation2d:
        return self.navx.getRotation2d()
    
    def drive(self, chassis_speed:ChassisSpeeds, field_relative: bool=True) -> None:
        # Shoutout to team 1706, your code saved our swerve this year lmao
        # Insert function to steady target angle here :)))

        if field_relative:
            if RobotBase.isReal():
                states = self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassis_speed, self.get_angle()))
            else:
                states = self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassis_speed, Rotation2d.fromDegrees(self.target_rad)))
        else:
            states = self.kinematics.toSwerveModuleStates(chassis_speed)

        desat_states = self.kinematics.desaturateWheelSpeeds(states, Waffles.k_max_speed)

        self.chassis_speed = chassis_speed

        self.set_module_states(desat_states)
        
        if not RobotBase.isReal():
            self.target_rad += chassis_speed.omega / 50
            
    def hockey_stop(self) -> None:
        self.left_front.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(225)))
        self.left_rear.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.right_front.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(135)))
        self.right_rear.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def get_chassis_speeds(self) -> ChassisSpeeds:
        return self.chassis_speed

    def set_module_states(self, module_states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        desatStates = self.kinematics.desaturateWheelSpeeds(module_states, Waffles.k_max_speed)

        self.left_front.set_desired_state(desatStates[0])
        self.left_rear.set_desired_state(desatStates[1])
        self.right_front.set_desired_state(desatStates[2])
        self.right_rear.set_desired_state(desatStates[3])

    def get_pose(self) -> Pose2d:
        return self.odometry.getPose()

    def reset_odometry(self, pose=Pose2d()) -> None:
        self.target_rad = 0
        self.odometry.resetPosition(self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()), pose)
        
    def reset_odometry_command(self) -> Command:
        return self.runOnce(lambda: self.reset_odometry())

    def periodic(self) -> None:
        if RobotBase.isReal():
            self.odometry.update(self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()))
        else:
            self.odometry.update(Rotation2d(self.target_rad), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()))
        self.field.setRobotPose(self.odometry.getPose())
        SmartDashboard.putNumber("test", self.left_front.get_position().distance)
        SmartDashboard.putData(self.field)
        
    def initialize(self) -> None:
        self.navx.reset()
        
        self.left_front.reset_sensor_position()
        self.left_rear.reset_sensor_position()
        self.right_front.reset_sensor_position()
        self.right_rear.reset_sensor_position()

"""
CONVERSIONS
"""

def meters_to_rots(meters: float, ratio: float) -> float:
    """Converts from the inserted amount of meters to wheel rotations. 
    This can also be used to convert from velocity in m/s to rps, as well as acceleration in m/s^2 to rps/s

    Args:
        meters (float): Target in meters.

    Returns:
        float: Converted amount of rotations. This is multiplied by the mechanism gear ratio.
    """
    wheelCircum = math.pi * Waffles.k_wheel_size
    return (meters / wheelCircum) * ratio

def rots_to_meters(rotation: float, ratio: float=1) -> float:
    """Converts from the applied TalonFX rotations and calculates the amount of meters traveled.
    This can also be used to convert from velocity in rps to m/s, as well as acceleration in rps/s to m/s^2

    Args:
        rotation (float): TalonFX rotations

    Returns:
        float: Meters traveled
    """
    baseMotorRot = rotation / ratio
    wheelCircum = math.pi * Waffles.k_wheel_size
    return baseMotorRot * wheelCircum

def rots_to_degs(rotation: float) -> float:
    """Converts from the rotations of the mechanism to degs rotated.

    Args:
        rotation (float): Rotation of the specified motor.

    Returns:
        float: Degrees the wheel has rotated.
    """
    return rotation * 360

def degs_to_rots(degrees: float) -> float:
    """Converts from degrees to TalonFX rotations.

    Args:
        degrees (float): Target degrees.

    Returns:
        float: Rotations of the TalonFX.
    """
    return degrees / 360
