from math import pi, sqrt

from commands2 import InstantCommand, Subsystem
import math
import navx
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig
from phoenix6.configs.cancoder_configs import *
from phoenix6.configs.talon_fx_configs import *
from phoenix6.controls import *
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls.motion_magic_voltage import MotionMagicVoltage
from phoenix6.signals import *
from phoenix6 import BaseStatusSignal
from typing import Self
from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard
from wpilib.shuffleboard import Shuffleboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState

from constants import *


class SwerveModule(Subsystem):


    def __init__(self, drive_id: int, direction_id: int, CANcoder_id: int, CAN_offset: float, tab_name: str, dir_inverted: bool=False) -> None:
        super().__init__()

        self.turning_encoder = CANcoder(CANcoder_id, "rio")
        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        encoder_config.magnet_sensor.magnet_offset = CAN_offset
        encoder_config.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        self.turning_encoder.configurator.apply(encoder_config)
        
        self.drive_motor = TalonFX(drive_id, "rio")
        drive_configs = TalonFXConfiguration()
        drive_configs.slot0 = Slot0Configs().with_k_p(DriveMotorConstants.kP).with_k_a(DriveMotorConstants.kA).with_k_s(DriveMotorConstants.kS)
        drive_configs.motor_output.neutral_mode = DriveMotorConstants.kNeutral
        drive_configs.motor_output.inverted = DriveMotorConstants.kInvert
        drive_configs.feedback.sensor_to_mechanism_ratio = DriveMotorConstants.kRatio
        self.drive_motor.configurator.apply(drive_configs)

        self.direction_motor = TalonFX(direction_id, "rio")
        self.direction_motor.set_position(0)
        steer_configs = TalonFXConfiguration()
        steer_configs.slot0 = Slot0Configs().with_k_p(SteerMotorConstants.kP).with_k_i(SteerMotorConstants.kI).with_k_s(SteerMotorConstants.kS)
        steer_configs.motor_output.neutral_mode = SteerMotorConstants.kNeutral
        if dir_inverted:
            steer_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        steer_configs.motor_output.inverted = SteerMotorConstants.kInvert
        
        if RobotBase.isReal():
            steer_configs.feedback.feedback_remote_sensor_id = CANcoder_id
            steer_configs.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        steer_configs.feedback.rotor_to_sensor_ratio = SteerMotorConstants.kRatio
        
        steer_configs.closed_loop_general.continuous_wrap = True
        self.direction_motor.configurator.apply(steer_configs)

        self.drive_position = self.drive_motor.get_position()
        self.drive_velocity = self.drive_motor.get_velocity()
        self.steer_position = self.direction_motor.get_position()
        self.steer_velocity = self.direction_motor.get_velocity()

        self.signals = [self.drive_position, self.drive_velocity, self.steer_position, self.steer_velocity]
        
        self.angle_setter = PositionDutyCycle(0)
        self.velocity_setter = VelocityTorqueCurrentFOC(0)

        self.internal_state = SwerveModulePosition()

        self.tab_name = tab_name

        self.speed_multiplier = 1

    def refresh(self) -> None:
        map(lambda signal: signal.refresh(), self.signals)

    def get_signals(self):
        return self.signals

    def get_angle(self, refresh=True) -> Rotation2d:
        if refresh:
            self.refresh()

        #steer_compensated = self.steer_position.value #BaseStatusSignal.get_latency_compensated_value(self.steer_position, self.steer_velocity)
        SmartDashboard.putNumber(f"{self.drive_motor.device_id} testAngle", rots_to_degs(self.direction_motor.get_position().value % 360))
        return Rotation2d.fromDegrees(rots_to_degs(self.direction_motor.get_position().value))

    def get_state(self, refresh=True) -> SwerveModuleState:
        if refresh:
            self.refresh()
        
        return SwerveModuleState(rots_to_meters(self.drive_velocity.value), self.get_angle())
    
    def get_position(self, refresh=True) -> SwerveModulePosition:
        if refresh:
            self.refresh()

        drive_compensated = BaseStatusSignal.get_latency_compensated_value(self.drive_position, self.drive_velocity)

        self.internal_state = SwerveModulePosition(rots_to_meters(drive_compensated), self.get_angle())
        
        return self.internal_state
    
    def _invert_drive(self) -> None:
        self.speed_multiplier *= -1

    def set_desired_state(self, desiredState: SwerveModuleState, override_brake_dur_neutral: bool=True) -> None:
        self.refresh()
        desiredAngle = desiredState.angle.degrees() % 360

        angleDist = math.fabs(desiredAngle - self.internal_state.angle.degrees())

        if (angleDist > 90 and angleDist < 270):
            desiredAngle = (desiredAngle + 180) % 360
            self._invert_drive()

        self.direction_motor.set_control(self.angle_setter.with_position(degs_to_rots(desiredAngle)))
        SmartDashboard.putNumber(f"{self.drive_motor.device_id} Control Speed", desiredState.speed * self.speed_multiplier)
        self.drive_motor.set_control(self.velocity_setter.with_velocity(meters_to_rots(desiredState.speed * self.speed_multiplier)).with_override_coast_dur_neutral(override_brake_dur_neutral))
        

class Swerve(Subsystem):
    navx = navx.AHRS.create_spi()
    navx.enableLogging(False)

    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1), Translation2d(1, -1), Translation2d(-1, -1)) # LF, LR, RF, RR
    
    field = Field2d()
    
    left_front: SwerveModule = SwerveModule(MotorIDs.LEFT_FRONT_DRIVE, MotorIDs.LEFT_FRONT_DIRECTION, CANIDs.LEFT_FRONT, CANOffsets.LEFT_FRONT, "Left Front", True)
    left_rear: SwerveModule = SwerveModule(MotorIDs.LEFT_REAR_DRIVE, MotorIDs.LEFT_REAR_DIRECTION, CANIDs.LEFT_REAR, CANOffsets.LEFT_REAR, "Left Rear", True)
    right_front: SwerveModule = SwerveModule(MotorIDs.RIGHT_FRONT_DRIVE, MotorIDs.RIGHT_FRONT_DIRECTION, CANIDs.RIGHT_FRONT, CANOffsets.RIGHT_FRONT, "Right Front")
    right_rear: SwerveModule = SwerveModule(MotorIDs.RIGHT_REAR_DRIVE, MotorIDs.RIGHT_REAR_DIRECTION, CANIDs.RIGHT_REAR, CANOffsets.RIGHT_REAR, "Right Rear")
    modules = [left_front, left_rear, right_front, right_rear]
    
    def __init__(self):
        super().__init__()

        self.odometry = SwerveDrive4PoseEstimator(self.kinematics, self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()), Pose2d())

        SmartDashboard.putData(self.field)
        reset_yaw = InstantCommand(lambda: self.reset_yaw())
        reset_yaw.setName("Reset Yaw")
        SmartDashboard.putData("Reset Gyro", reset_yaw)
        
        self.set_max_module_speed()
        
        if not AutoBuilder.isConfigured():
            AutoBuilder.configureHolonomic(
                lambda: self.get_pose(),
                lambda pose: self.reset_odometry(pose),
                lambda: self.get_robot_relative_speeds(),
                lambda chassisSpeed: self.robot_centric_drive(chassisSpeed),
                HolonomicPathFollowerConfig(
                    PIDConstants(5.0, 0.0, 0.0, 0.0), # translation
                    PIDConstants(5.0, 0.0, 0.0, 0.0), # rotation
                    Waffles.k_max_module_speed,
                    Waffles.k_drive_base_radius,
                    ReplanningConfig()
                ),
                self.should_flip_auto_path,
                self
            )
        
        self.navx.reset()
        self.obdn = True
        
        all_signals: list[BaseStatusSignal] = []
        for module in Swerve.modules:
            for signal in module.get_signals():
                all_signals.append(signal)
        map(lambda signal: signal.set_update_frequency(250), all_signals)
        
    def should_flip_auto_path(self) -> bool:
        if RobotBase.isReal():
            return DriverStation.getAlliance() == DriverStation.Alliance.kRed
        else:
            return False

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.navx.getYaw())
    
    def field_relative_drive(self, chassis_speed: ChassisSpeeds, center_of_rotation: Translation2d=Translation2d()) -> None: # Discretizes the chassis speeds, then transforms it into individual swerve module states (field relative)
        self.set_module_states(self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds.discretize(chassis_speed, 0.02), self.get_angle()), centerOfRotation=center_of_rotation))
        
    def get_field_relative_speeds(self) -> ChassisSpeeds:
        return ChassisSpeeds.fromRobotRelativeSpeeds(self.get_robot_relative_speeds(), self.get_angle())
        
    def robot_centric_drive(self, chassis_speed: ChassisSpeeds, center_of_rotation: Translation2d=Translation2d()) -> None: # see drive(), but less cool to watch
        self.set_module_states(self.kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassis_speed, 0.02)))
        
    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds((self.left_front.get_state(), self.left_rear.get_state(), self.right_front.get_state(), self.right_rear.get_state()))

    def set_module_states(self, module_states: tuple) -> None:
        desatStates = self.kinematics.desaturateWheelSpeeds(module_states, self.max_module_speed)
        
        i = 0
        for module in self.modules:
            module.set_desired_state(desatStates[i], override_brake_dur_neutral=self.obdn)
            i += 1
        
    def set_max_module_speed(self, max_module_speed: float=Waffles.k_max_module_speed) -> None:
        self.max_module_speed = max_module_speed
        
    def set_module_override_brake(self, new_obdn: bool) -> None:
        self.obdn = new_obdn

    def set_voltage(self, volts: float) -> None:
        """For SysId tuning"""
        steady_dir = self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds.discretize(volts, 0, 0, 0.02), self.get_angle()))
        
        self.left_front.direction_motor.set_control(MotionMagicVoltage(degs_to_rots(steady_dir[0].angle.degrees())))
        self.left_rear.direction_motor.set_control(MotionMagicVoltage(degs_to_rots(steady_dir[1].angle.degrees())))
        self.right_front.direction_motor.set_control(MotionMagicVoltage(degs_to_rots(steady_dir[2].angle.degrees())))
        self.left_front.direction_motor.set_control(MotionMagicVoltage(degs_to_rots(steady_dir[3].angle.degrees())))
        
        self.left_front.drive_motor.set_control(VoltageOut(volts, override_brake_dur_neutral=True))
        self.left_rear.drive_motor.set_control(VoltageOut(volts, override_brake_dur_neutral=True))
        self.right_front.drive_motor.set_control(VoltageOut(volts, override_brake_dur_neutral=True))
        self.right_rear.drive_motor.set_control(VoltageOut(volts, override_brake_dur_neutral=True))
        
    def log_motor_output(self, log: SysIdRoutineLog) -> None: # Unsued since we just convert the hoot file
        pass

    def get_pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def reset_odometry(self, pose=Pose2d()) -> None:
        self.odometry.resetPosition(self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()), pose)
    
    def reset_yaw(self) -> Self:
        self.navx.reset()
        return self

    def periodic(self) -> None:
        map(lambda module: module.refresh(), self.modules)
        self.field.setRobotPose(self.odometry.update(self.get_angle(), (self.left_front.get_position(False), self.left_rear.get_position(False), self.right_front.get_position(False), self.right_rear.get_position(False))))
        
        SmartDashboard.putData(self.field)
        SmartDashboard.putNumber("Gyro", -self.get_angle().degrees())
        SmartDashboard.putBoolean("NavX Connection", self.navx.isConnected())

    def addVisionMeasurement(self, pose: Pose2d, timestamp: float) -> None:
        current_pose = self.odometry.getEstimatedPosition()
        if sqrt((current_pose.X() - pose.X())**2 + (current_pose.Y() - pose.Y())**2) > 1:
            return
        self.odometry.addVisionMeasurement(pose, timestamp)

"""
CONVERSIONS
"""

def meters_to_rots(meters: float) -> float:
    return meters / (pi * Waffles.k_wheel_size)

def rots_to_meters(rotation: float) -> float:
    return rotation * (pi * Waffles.k_wheel_size)

def rots_to_degs(rotation: float) -> float:
    return rotation * 360

def degs_to_rots(degrees: float) -> float:
    return degrees / 360
