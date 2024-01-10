from commands2 import SubsystemBase
from constants import *
from ctre import *
from ctre.sensors import CANCoder, SensorInitializationStrategy
import math
from wpimath.kinematics import SwerveModuleState
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d

class SwerveModule(SubsystemBase):
    """
    Takes inputted SwerveModuleStates and moves the direction and drive motor to the selected positions.

    The direction motor rotates the wheel into position.
    The drive motor spins the wheel to move.
    """
    driveFeedForward = SimpleMotorFeedforwardMeters(kA=0, kV=0, kS=0)
    
    def __init__(self, moduleName: str, directionMotorController: TalonFX, driveMotorController: TalonFX, CANCoder: CANCoder, offset: float) -> None:
        super().__init__()

        self.moduleName = moduleName

        self.directionMotor = directionMotorController
        self.directionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Motor.kTimeoutMs)
        
        self.directionMotor.config_kP(Motor.kSlotIdx, DirectionMotor.kP, Motor.kTimeoutMs)
        self.directionMotor.config_kI(Motor.kSlotIdx, DirectionMotor.kI, Motor.kTimeoutMs)
        self.directionMotor.config_kD(Motor.kSlotIdx, DirectionMotor.kD, Motor.kTimeoutMs)
        self.directionMotor.config_kF(Motor.kSlotIdx, DirectionMotor.kF, Motor.kTimeoutMs)
        self.directionMotor.configMotionCruiseVelocity(DirectionMotor.kCruiseVel, Motor.kTimeoutMs)
        self.directionMotor.configMotionAcceleration(DirectionMotor.kCruiseAccel, Motor.kTimeoutMs)

        self.directionMotor.configVoltageCompSaturation(Motor.kVoltCompensation, Motor.kTimeoutMs)
        self.directionMotor.setInverted(False)
        self.directionMotor.setNeutralMode(NeutralMode.Brake)

        self.driveMotor = driveMotorController
        self.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Motor.kTimeoutMs)

        self.driveMotor.config_kP(Motor.kSlotIdx, DriveMotor.kP, Motor.kTimeoutMs)
        self.driveMotor.config_kI(Motor.kSlotIdx, DriveMotor.kI, Motor.kTimeoutMs)
        self.driveMotor.config_kD(Motor.kSlotIdx, DriveMotor.kD, Motor.kTimeoutMs)

        self.driveMotor.configVoltageCompSaturation(Motor.kVoltCompensation, Motor.kTimeoutMs)
        self.driveMotor.setInverted(False)
        self.driveMotor.setNeutralMode(NeutralMode.Brake)

        self.distancePerPulse = math.tau * Larry.kWheelSize / 4096
        self.turningPerPulse = math.tau / 4096

        self.turningEncoder = CANCoder
        self.turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Motor.kTimeoutMs)
        self.turningEncoder.configSensorDirection(True, Motor.kTimeoutMs)
        self.turningEncoder.configMagnetOffset(offset)

    def getState(self) -> SwerveModuleState:
        # units/100ms -> m/s
        speed = self.driveMotor.getSelectedSensorVelocity() / Motor.kGearRatio * 10 * Larry.kWheelSize * math.pi
        rotation = self.directionMotor.getSelectedSensorPosition() / Motor.kGearRatio * (360/2048)
        return SwerveModuleState(speed, Rotation2d.fromDegrees(rotation))
        