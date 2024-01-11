from commands2 import SubsystemBase
from constants import *
from ctre import *
from ctre.sensors import CANCoder, SensorInitializationStrategy
import math
import navx
from wpimath.kinematics import SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry
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

    def getState(self) -> SwerveModuleState:
        # units/100ms -> m/s
        speed = self.driveMotor.getSelectedSensorVelocity() / Motor.kGearRatio * 10 * Larry.kWheelSize * math.pi
        rotation = self.directionMotor.getSelectedSensorPosition() / Motor.kGearRatio * (360/2048)
        return SwerveModuleState(speed, Rotation2d.fromDegrees(rotation))

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        currentState = self.getState()
        state = SwerveModuleState.optimize(desiredState, currentState.angle)

        self.driveMotor.set(ControlMode.Current, self.driveFeedForward.calculate(currentState.speed, state.speed, 0.02) / 120)
        self.directionMotor.set(ControlMode.Position, state * (180/math.pi) * (2048/360))

""""""

class Swerve(SubsystemBase):
    leftFront = SwerveModule("LF", MotorIDs.LEFT_FRONT_DIRECTION, MotorIDs.LEFT_FRONT_DRIVE, CANIDs.LEFT_FRONT, COffsets.kFLOffset)
    leftRear = SwerveModule("LR", MotorIDs.LEFT_REAR_DIRECTION, MotorIDs.LEFT_REAR_DRIVE, CANIDs.LEFT_REAR, COffsets.kFROffset)
    rightFront = SwerveModule("RF", MotorIDs.RIGHT_FRONT_DIRECTION, MotorIDs.RIGHT_FRONT_DRIVE, CANIDs.RIGHT_FRONT, COffsets.kFROffset)
    rightRear = SwerveModule("RR", MotorIDs.RIGHT_REAR_DIRECTION, MotorIDs.RIGHT_REAR_DRIVE, CANIDs.RIGHT_REAR, COffsets.kRROffset)

    navX = navx.AHRS.create_spi()

    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1),
                                        Translation2d(1, -1), Translation2d(-1, -1)) # LF, LR, RF, RR
    
    def __init__(self) -> None:
        super().__init__()

        self.odometry = SwerveDrive4Odometry(self.kinematics, self.getAngle())

    def getAngle(self) -> float:
        return self.navX.getYaw()

    




        