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
from wpimath.controller import PIDController
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

    def __init__(self, moduleName: str, directionMotorControllerID: int, driveMotorControllerID: int, CANCoderID: int, offset: float) -> None:
        super().__init__()
        CommandScheduler.getInstance().registerSubsystem(self)

        self.moduleName = moduleName

        self.turningEncoder = CANcoder(CANCoderID, "rio")
        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor = MagnetSensorConfigs().with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE)
        self.turningEncoder.configurator.apply(encoder_config)

        # Direction Motor, uses MotionMagicTorqueCurrentFOC
        self.directionMotor = TalonFX(directionMotorControllerID, "rio")
        direction_config = TalonFXConfiguration()
        direction_config.motor_output.with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE).with_neutral_mode(NeutralModeValue.COAST)
        
        # These are used for angle optimizing, now built in with Phoenix 6 :partying_face:
        direction_config.closed_loop_general.continuous_wrap = True
        direction_config.feedback.sensor_to_mechanism_ratio = Motor.kGearRatio
        
        direction_config.motion_magic.motion_magic_acceleration = DirectionMotor.kCruiseAccel
        direction_config.motion_magic.motion_magic_cruise_velocity = DirectionMotor.kCruiseVel
        direction_slot0 = Slot0Configs() # See drive motor config below for explanation for each of these!!!
        direction_slot0.k_s = 0
        direction_slot0.k_a = 1
        direction_slot0.k_p = 0
        direction_slot0.k_i = 0
        direction_slot0.k_d = 0
        
        direction_config.slot0 = direction_slot0
        self.directionMotor.configurator.apply(direction_config)

        # Drive Motor, uses VelocityTorqueCurrentFOC
        self.driveMotor = TalonFX(driveMotorControllerID, "rio")
        drive_config = TalonFXConfiguration()
        drive_config.motor_output.with_inverted(InvertedValue.CLOCKWISE_POSITIVE).with_neutral_mode(NeutralModeValue.BRAKE)
        
        drive_slot0 = Slot0Configs()
        drive_slot0.k_s = 0 # Amount of volts to overcome friction (just below the required voltage to start moving)
        drive_slot0.k_a = 1 # Yeah uh we don't know yet-
        
        drive_slot0.k_v = 0 # Voltage needed to rotate the wheel at 1 rps (i think), e.g. 0.11 = 0.11V to move the wheel at 1 rps 
        # NOTE: MAY BE UNEEDED DUE TO USING TORQUECURRENTFOC, SEE 
        # https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#choosing-output-type
        
        drive_slot0.k_p = 0
        drive_slot0.k_i = 0
        drive_slot0.k_d = 0
        
        drive_config.slot0 = drive_slot0
        self.driveMotor.configurator.apply(drive_config)

    def getAngle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(rotsToDegs(self.directionMotor.get_rotor_position().value))
    
    def resetSensorPosition(self) -> None:
        pos = -self.turningEncoder.get_absolute_position().value
        self.directionMotor.set_position(pos * Motor.kGearRatio)
        
    def resetPositions(self) -> None:
        self.driveMotor.set_position(0.0)
        self.directionMotor.set_position(0.0)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(rotsToMeters(self.driveMotor.get_rotor_velocity().value), self.getAngle())
    
    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(rotsToMeters(self.driveMotor.get_rotor_position().value), self.getAngle())

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        desiredState = SwerveModuleState.optimize(desiredState, self.getState().angle)
        
        self.driveMotor.set_control(VelocityTorqueCurrentFOC(metersToRots(desiredState.speed)))
        self.directionMotor.set_control(MotionMagicTorqueCurrentFOC(degsToRots(desiredState.angle.degrees())))

""""""

class Swerve(Subsystem):
    anglePID = PIDController(0, 0, 0)

    navX = navx.AHRS.create_spi()

    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1),
                                        Translation2d(1, -1), Translation2d(-1, -1)) # LF, LR, RF, RR
    
    field = Field2d()
    
    leftFront: SwerveModule = SwerveModule("LF", MotorIDs.LEFT_FRONT_DIRECTION, MotorIDs.LEFT_FRONT_DRIVE, CANIDs.LEFT_FRONT, CANOffsets.kLeftFrontOffset)
    leftRear: SwerveModule = SwerveModule("LR", MotorIDs.LEFT_REAR_DIRECTION, MotorIDs.LEFT_REAR_DRIVE, CANIDs.LEFT_REAR, CANOffsets.kLeftRearOffset)
    rightFront: SwerveModule = SwerveModule("RF", MotorIDs.RIGHT_FRONT_DIRECTION, MotorIDs.RIGHT_FRONT_DRIVE, CANIDs.RIGHT_FRONT, CANOffsets.kRightFrontOffset)
    rightRear: SwerveModule = SwerveModule("RR", MotorIDs.RIGHT_REAR_DIRECTION, MotorIDs.RIGHT_REAR_DRIVE, CANIDs.RIGHT_REAR, CANOffsets.kRightRearOffset)
    
    def __init__(self):
        super().__init__()
        CommandScheduler.getInstance().registerSubsystem(self)

        self.odometry = SwerveDrive4Odometry(self.kinematics, self.getAngle(),
                                             (self.leftFront.getPosition(), self.leftRear.getPosition(),
                                              self.rightFront.getPosition(), self.rightRear.getPosition()))

        SmartDashboard.putData(self.field)
        SmartDashboard.putData("Reset Odometry", self.resetOdometryCommand())

        self.chassisSpeed = ChassisSpeeds()
        self.targetRad = 0
        
        # https://pathplanner.dev/pplib-getting-started.html#holonomic-swerve
        AutoBuilder.configureHolonomic(
            lambda: self.getPose(),
            lambda pose: self.resetOdometry(pose),
            lambda: self.getChassisSpeeds(),
            lambda chassisSpeed: self.drive(chassisSpeed, fieldRelative=False),
            HolonomicPathFollowerConfig(
                PIDConstants(0.0, 0.0, 0.0, 0.0), # translation
                PIDConstants(0.0, 0.0, 0.0, 0.0), # rotation
                Larry.kMaxSpeed / 4,
                Larry.kDriveBaseRadius,
                ReplanningConfig(enableInitialReplanning=False)
            ),
            lambda: self.shouldFlipAutoPath(),
            self
        )
        
        self.navX.reset()

    def shouldFlipAutoPath(self) -> bool:
        # Flips the PathPlanner path if we're on the red alliance
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def runAuto(self, auto: PathPlannerAuto) -> None:
        self.runOnce(lambda: auto)

    def initialize(self) -> None:
        self.leftFront.resetSensorPosition()
        self.leftRear.resetSensorPosition()
        self.rightFront.resetSensorPosition()
        self.rightRear.resetSensorPosition()
        self.navX.reset()

    def getAngle(self) -> Rotation2d:
        return self.navX.getRotation2d()
    
    def drive(self, chassisSpeed:ChassisSpeeds, fieldRelative: bool=True) -> None:
        # Shoutout to team 1706, your code saved our swerve this year lmao
        # Insert function to steady target angle here :)))

        if fieldRelative:
            if RobotBase.isReal():
                states = self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeed, self.getAngle()))
            else:
                states = self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeed, Rotation2d.fromDegrees(self.targetRad)))
        else:
            states = self.kinematics.toSwerveModuleStates(chassisSpeed)

        desatStates = self.kinematics.desaturateWheelSpeeds(states, Larry.kMaxSpeed)

        self.chassisSpeed = chassisSpeed

        self.setModuleStates(desatStates)
        
        if not RobotBase.isReal():
            self.targetRad += chassisSpeed.omega / 50
            
    def hockeyStop(self) -> None:
        self.leftFront.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(225)))
        self.leftRear.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rightFront.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(135)))
        self.rightRear.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def getChassisSpeeds(self) -> ChassisSpeeds:
        return self.chassisSpeed

    def setModuleStates(self, moduleStates: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState], optimizeAngle=True) -> None:
        desatStates = self.kinematics.desaturateWheelSpeeds(moduleStates, Larry.kMaxSpeed)

        self.leftFront.setDesiredState(desatStates[0])
        self.leftRear.setDesiredState(desatStates[1])
        self.rightFront.setDesiredState(desatStates[2])
        self.rightRear.setDesiredState(desatStates[3])

    def getPose(self) -> Pose2d:
        return self.odometry.getPose()

    def resetOdometry(self, pose=Pose2d()) -> None:
        self.targetRad = 0
        self.odometry.resetPosition(self.getAngle(), (self.leftFront.getPosition(), self.leftRear.getPosition(), self.rightFront.getPosition(), self.rightRear.getPosition()), pose)
        
    def resetOdometryCommand(self) -> Command:
        return self.runOnce(lambda: self.resetOdometry())

    def periodic(self) -> None:
        if RobotBase.isReal():
            self.odometry.update(self.getAngle(), (self.leftFront.getPosition(), self.leftRear.getPosition(), self.rightFront.getPosition(), self.rightRear.getPosition()))
        else:
            self.odometry.update(Rotation2d(self.targetRad), (self.leftFront.getPosition(), self.leftRear.getPosition(), self.rightFront.getPosition(), self.rightRear.getPosition()))
        self.field.setRobotPose(self.odometry.getPose())
        SmartDashboard.putData(self.field)
        
    def initialize(self) -> None:
        self.navX.reset()
        
        self.leftFront.resetPositions()
        self.leftRear.resetPositions()
        self.rightFront.resetPositions()
        self.rightRear.resetPositions()
        
        self.leftFront.resetSensorPosition()
        self.leftRear.resetSensorPosition()
        self.rightFront.resetSensorPosition()
        self.rightRear.resetSensorPosition()

"""
CONVERSIONS
"""

def metersToRots(meters: float) -> float:
    """Converts from the inserted amount of meters to wheel rotations. 
    This can also be used to convert from velocity in m/s to rps, as well as acceleration in m/s^2 to rps/s

    Args:
        meters (float): Target in meters.

    Returns:
        float: Converted amount of rotations. This is multiplied by the mechanism gear ratio.
    """
    wheelCircum = math.pi * Larry.kWheelSize
    return (meters / wheelCircum) * Motor.kGearRatio

def rotsToMeters(rotation: float) -> float:
    """Converts from the applied TalonFX rotations and calculates the amount of meters traveled.
    This can also be used to convert from velocity in rps to m/s, as well as acceleration in rps/s to m/s^2

    Args:
        rotation (float): TalonFX rotations

    Returns:
        float: Meters traveled
    """
    baseMotorRot = rotation / Motor.kGearRatio
    wheelCircum = math.pi * Larry.kWheelSize
    return baseMotorRot * wheelCircum

def rotsToDegs(rotation: float) -> float:
    """Converts from the rotations of the mechanism to degs rotated.

    Args:
        rotation (float): Rotation of the specified motor.

    Returns:
        float: Degrees the wheel has rotated.
    """
    return rotation * 360

def degsToRots(degrees: float) -> float:
    """Converts from degrees to TalonFX rotations.

    Args:
        degrees (float): Target degrees.

    Returns:
        float: Rotations of the TalonFX.
    """
    return degrees / 360
