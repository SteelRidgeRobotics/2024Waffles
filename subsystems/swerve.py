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

        self.moduleName = moduleName

        self.turningEncoder = CANcoder(CANCoderID, "rio")
        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor = MagnetSensorConfigs().with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE)
        self.turningEncoder.configurator.apply(encoder_config)

        self.directionMotor = TalonFX(directionMotorControllerID, "rio")
        direction_config = TalonFXConfiguration()
        direction_config.motor_output.with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE).with_neutral_mode(NeutralModeValue.COAST)
        direction_config.motion_magic.with_motion_magic_cruise_velocity(DirectionMotor.kCruiseVel).with_motion_magic_acceleration(DirectionMotor.kCruiseAccel).with_motion_magic_jerk(1600)
        direction_config.slot0 = Slot0Configs().with_k_v(DirectionMotor.kF).with_k_p(DirectionMotor.kP).with_k_i(DirectionMotor.kI).with_k_d(DirectionMotor.kD)
        self.directionMotor.configurator.apply(direction_config)
        
        self.motionMagic = MotionMagicVoltage(0)
        
        self.directionMotor.set_position(0.0)

        self.driveMotor = TalonFX(driveMotorControllerID, "rio")
        drive_config = TalonFXConfiguration()
        drive_config.motor_output.with_inverted(InvertedValue.CLOCKWISE_POSITIVE).with_neutral_mode(NeutralModeValue.BRAKE)
        drive_config.slot0 = Slot0Configs()
        self.driveMotor.configurator.apply(drive_config)
        
        self.arbFF = DriveMotor.karbFF
        
        self.simDrivePos = 0

        CommandScheduler.getInstance().registerSubsystem(self)

    def getAngle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.directionMotor.get_rotor_position().value / Motor.kGearRatio * 360)
    
    def resetSensorPosition(self) -> None:
        pos = -self.turningEncoder.get_absolute_position().value
        self.directionMotor.set_position(pos * Motor.kGearRatio)
        
    def resetPositions(self) -> None:
        self.driveMotor.set_position(0.0)
        self.directionMotor.set_position(0.0)

    def getState(self) -> SwerveModuleState:
        # units/100ms -> m/s
        speed = self.driveMotor.get_rotor_velocity().value / Motor.kGearRatio * Larry.kWheelSize * math.pi
        rotation = self.directionMotor.get_rotor_position().value / Motor.kGearRatio * 360
        return SwerveModuleState(speed, Rotation2d.fromDegrees(rotation))
    
    def getPosition(self) -> SwerveModulePosition:
        if not RobotBase.isReal():
            return SwerveModulePosition(self.simDrivePos, self.getAngle())
        else:
            return SwerveModulePosition(
                (self.driveMotor.get_rotor_position().value / Motor.kGearRatio) * (Larry.kWheelSize*math.pi),
                self.getAngle()
            )
        
    def simulationPeriodic(self) -> None:
        self.simDrivePos += self.driveMotor.get_rotor_velocity().value * (1 / Motor.kGearRatio) * (Larry.kWheelSize * math.pi)
        
    def periodic(self) -> None:
        SmartDashboard.putNumber(self.moduleName + "encoderRot", self.turningEncoder.get_absolute_position().value)

    def setDesiredState(self, desiredState: SwerveModuleState, optimize=True) -> None:
        currentState = self.getState()
        if optimize:
            desiredState = SwerveModuleState.optimize(desiredState, currentState.angle)
        
        if desiredState.speed < 0:
            arbFF = -self.arbFF
        else:
            arbFF = self.arbFF

        #self.driveMotor.set(ControlMode.PercentOutput, desiredState.speed / Larry.kMaxSpeed, DemandType.ArbitraryFeedForward, arbFF)
        SmartDashboard.putNumber(self.moduleName + "DutyCycleOut", desiredState.speed / Larry.kMaxSpeed)
        self.driveMotor.set_control(DutyCycleOut(desiredState.speed / Larry.kMaxSpeed))

        self.changeDirection(desiredState.angle)

    def changeDirection(self, rotation: Rotation2d) -> None:
        angleDiff = rotation.degrees() - self.getAngle().degrees()
        targetAngleDist = math.fabs(angleDiff)

        # When going from x angle to 0, the robot will try and go "the long way around" to the angle. This just checks to make sure we're actually getting the right distance
        if targetAngleDist > 180:
            while targetAngleDist > 180:
                targetAngleDist -= 360
            targetAngleDist = abs(targetAngleDist)

        changeInRots = targetAngleDist / 360

        if angleDiff < 0 or angleDiff >= 360:
            angleDiff %= 360
        
        finalPos = self.directionMotor.get_rotor_position().value / Motor.kGearRatio
        if angleDiff > 180:
            # Move CCW
            finalPos -= changeInRots
        else:
            # Move CW
            finalPos += changeInRots

        self.motionMagic.slot = 0
        self.directionMotor.set_control(self.motionMagic.with_position(finalPos * Motor.kGearRatio))
        self.directionMotor.sim_state.add_rotor_position(changeInRots)

""""""

class Swerve(Subsystem):
    anglePID = PIDController(0, 0, 0)

    navX = navx.AHRS.create_spi()

    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1),
                                        Translation2d(1, -1), Translation2d(-1, -1)) # LF, LR, RF, RR
    
    field = Field2d()
    
    def __init__(self, leftFront: SwerveModule, leftRear: SwerveModule, rightFront: SwerveModule, rightRear: SwerveModule):
        super().__init__()

        self.leftFront = leftFront
        self.leftRear = leftRear
        self.rightFront = rightFront
        self.rightRear = rightRear

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

        CommandScheduler.getInstance().registerSubsystem(self)

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

        self.leftFront.setDesiredState(desatStates[0], optimize=optimizeAngle)
        self.leftRear.setDesiredState(desatStates[1], optimize=optimizeAngle)
        self.rightFront.setDesiredState(desatStates[2], optimize=optimizeAngle)
        self.rightRear.setDesiredState(desatStates[3], optimize=optimizeAngle)

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
        