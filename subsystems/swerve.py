from commands2 import Command, SubsystemBase
from constants import *
from ctre import *
from ctre.sensors import CANCoder, SensorInitializationStrategy
import math
import navx
from wpilib import Field2d, SmartDashboard
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Translation2d, Rotation2d

class SwerveModule(SubsystemBase):
    """
    Takes inputted SwerveModuleStates and moves the direction and drive motor to the selected positions.

    The direction motor rotates the wheel into position.
    The drive motor spins the wheel to move.
    """
    driveFeedForward = SimpleMotorFeedforwardMeters(kA=1, kV=1, kS=0)
    
    def __init__(self, moduleName: str, directionMotorControllerID: int, driveMotorControllerID: int, CANCoderID: int, offset: float) -> None:
        super().__init__()

        self.moduleName = moduleName

        self.directionMotor = TalonFX(directionMotorControllerID)
        self.directionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Motor.kTimeoutMs)
        self.directionMotor.selectProfileSlot(Motor.kSlotIdx, Motor.kPIDLoopIdx)
        
        self.directionMotor.config_kP(Motor.kSlotIdx, DirectionMotor.kP, Motor.kTimeoutMs)
        self.directionMotor.config_kI(Motor.kSlotIdx, DirectionMotor.kI, Motor.kTimeoutMs)
        self.directionMotor.config_kD(Motor.kSlotIdx, DirectionMotor.kD, Motor.kTimeoutMs)
        self.directionMotor.config_kF(Motor.kSlotIdx, DirectionMotor.kF, Motor.kTimeoutMs)
        self.directionMotor.configMotionCruiseVelocity(DirectionMotor.kCruiseVel, Motor.kTimeoutMs)
        self.directionMotor.configMotionAcceleration(DirectionMotor.kCruiseAccel, Motor.kTimeoutMs)

        self.directionMotor.configVoltageCompSaturation(Motor.kVoltCompensation, Motor.kTimeoutMs)
        self.directionMotor.setInverted(False)
        self.directionMotor.setNeutralMode(NeutralMode.Brake)

        self.driveMotor = TalonFX(driveMotorControllerID)
        self.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Motor.kTimeoutMs)
        self.driveMotor.selectProfileSlot(Motor.kSlotIdx, Motor.kPIDLoopIdx)

        self.driveMotor.config_kP(Motor.kSlotIdx, DriveMotor.kP, Motor.kTimeoutMs)
        self.driveMotor.config_kI(Motor.kSlotIdx, DriveMotor.kI, Motor.kTimeoutMs)
        self.driveMotor.config_kD(Motor.kSlotIdx, DriveMotor.kD, Motor.kTimeoutMs)

        self.driveMotor.configVoltageCompSaturation(Motor.kVoltCompensation, Motor.kTimeoutMs)
        self.driveMotor.setInverted(False)
        self.driveMotor.setNeutralMode(NeutralMode.Brake)

        self.distancePerPulse = math.tau * Larry.kWheelSize / 4096
        self.turningPerPulse = math.tau / 4096

        self.turningEncoder = CANCoder(CANCoderID)
        self.turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Motor.kTimeoutMs)
        self.turningEncoder.configSensorDirection(True, Motor.kTimeoutMs)
        self.turningEncoder.configMagnetOffset(offset)

        self.test = 0

    def getAngle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.directionMotor.getSelectedSensorPosition() / Motor.kGearRatio * (360/2048))

    def getState(self) -> SwerveModuleState:
        # units/100ms -> m/s
        speed = self.driveMotor.getSelectedSensorVelocity() / Motor.kGearRatio * 10 * Larry.kWheelSize * math.pi
        rotation = self.directionMotor.getSelectedSensorPosition() / Motor.kGearRatio * (360/2048)
        SmartDashboard.putNumber(self.moduleName + "rotation", rotation)
        return SwerveModuleState(speed, Rotation2d.fromDegrees(rotation))
    
    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.driveMotor.getSelectedSensorPosition() / Motor.kGearRatio * Larry.kWheelSize * math.pi,
            self.getAngle()
        )
    
    def simulationPeriodic(self) -> None:
        SmartDashboard.putNumber(self.moduleName + "drivePosNoGear", self.driveMotor.getSelectedSensorVelocity())
        SmartDashboard.putNumber(self.moduleName + "directionPos", self.directionMotor.getSelectedSensorPosition())

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        currentState = self.getState()
        state = SwerveModuleState.optimize(desiredState, currentState.angle)

        SmartDashboard.putNumber(self.moduleName + "state", state.angle.degrees())

        self.driveMotor.set(ControlMode.Current, self.driveFeedForward.calculate(currentState.speed, state.speed, 0.02) / 120)
        self.driveMotor.getSimCollection().setIntegratedSensorVelocity(int(state.speed / math.pi / Larry.kWheelSize / 10 * Motor.kGearRatio))

        self.directionMotor.set(ControlMode.Position, state.angle.degrees() * (2048/360) * Motor.kGearRatio)
        self.directionMotor.getSimCollection().setIntegratedSensorRawPosition(int(state.angle.degrees() * (2048/360) * Motor.kGearRatio))

""""""

class Swerve(SubsystemBase):
    anglePID = PIDController(0, 0, 0)

    navX = navx.AHRS.create_spi()

    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1),
                                        Translation2d(1, -1), Translation2d(-1, -1)) # LF, LR, RF, RR
    
    field = Field2d()
    
    def __init__(self, leftFront: SwerveModule, leftRear: SwerveModule, rightFront: SwerveModule, rightRear: SwerveModule) -> None:
        super().__init__()

        self.leftFront = leftFront
        self.leftRear = SwerveModule("LR", MotorIDs.LEFT_REAR_DIRECTION, MotorIDs.LEFT_REAR_DRIVE, CANIDs.LEFT_REAR, COffsets.kFROffset)
        self.rightFront = SwerveModule("RF", MotorIDs.RIGHT_FRONT_DIRECTION, MotorIDs.RIGHT_FRONT_DRIVE, CANIDs.RIGHT_FRONT, COffsets.kFROffset)
        self.rightRear = SwerveModule("RR", MotorIDs.RIGHT_REAR_DIRECTION, MotorIDs.RIGHT_REAR_DRIVE, CANIDs.RIGHT_REAR, COffsets.kRROffset)

        self.odometry = SwerveDrive4Odometry(self.kinematics, self.getAngle(),
                                             (self.leftFront.getPosition(), self.leftRear.getPosition(),
                                              self.rightFront.getPosition(), self.rightRear.getPosition()))
        self.targetAngle = 0

        SmartDashboard.putData(self.field)
        SmartDashboard.putData(self.resetOdometryCommand())

    def getAngle(self) -> float:
        return self.navX.getRotation2d()
    
    def drive(self, xSpeed: float, ySpeed: float, rotRate: float, fieldRelative: bool=True) -> None:
        # Shoutout to team 1706, your code saved our swerve this year lmao
        # Insert function to steady target angle here :)))
        SmartDashboard.putNumber("xSpeed", xSpeed)
        SmartDashboard.putNumber("ySpeed", ySpeed)
        SmartDashboard.putNumber("rotSpeed", rotRate)

        if fieldRelative:
            states = self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotRate, self.getAngle()))
        else:
            states = self.kinematics.toSwerveModuleStates(ChassisSpeeds(xSpeed, ySpeed, rotRate))

        desatStates = self.kinematics.desaturateWheelSpeeds(states, Larry.kMaxSpeed)
        SmartDashboard.putNumberArray("STATES", [desatStates[0].angle.degrees(), desatStates[1].angle.degrees(), desatStates[2].angle.degrees(), desatStates[3].angle.degrees()])

        self.setModuleStates(desatStates)

    def setModuleStates(self, moduleStates: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        desatStates = self.kinematics.desaturateWheelSpeeds(moduleStates, Larry.kMaxSpeed)

        self.leftFront.setDesiredState(desatStates[0])
        self.leftRear.setDesiredState(desatStates[1])
        self.rightFront.setDesiredState(desatStates[2])
        self.rightRear.setDesiredState(desatStates[3])

    def resetOdometry(self) -> None:
        self.odometry.resetPosition(self.getAngle(), self.odometry.getPose(),
                                    self.leftFront.getPosition(), self.leftRear.getPosition(), self.rightFront.getPosition(), self.rightRear.getPosition())

    def resetOdometryCommand(self) -> Command:
        return self.runOnce(lambda: self.resetOdometry())

    def periodic(self) -> None:
        self.field.setRobotPose(
            self.odometry.update(self.getAngle(), self.leftFront.getPosition(), self.leftRear.getPosition(), self.rightFront.getPosition(), self.rightRear.getPosition()))
        