from commands2 import SubsystemBase
from constants import *
from ctre import FeedbackDevice, TalonFX
from ctre.sensors import CANCoder, SensorInitializationStrategy
import math
from wpimath.kinematics import SwerveModuleState
from wpimath.controller import SimpleMotorFeedforwardMeters

class SwerveModule(SubsystemBase):
    """
    Takes inputted SwerveModuleStates and moves the direction and drive motor to the selected positions.

    The direction motor rotates the wheel into position.
    The drive motor spins the wheel to move.
    """
    driveFeedForward = SimpleMotorFeedforwardMeters()
    
    def __init__(self, moduleName: str, directionMotorController: TalonFX, driveMotorController: TalonFX, CANCoder: CANCoder, offset: float) -> None:
        super().__init__()

        self.moduleName = moduleName

        self.directionMotor = directionMotorController
        self.directionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Motor.kTimeoutMs)
        
        self.directionMotor.config_kP(Motor.kSlotIdx, DirectionMotor.kP, Motor.kTimeoutMs)
        self.directionMotor.config_kI(Motor.kSlotIdx, DirectionMotor.kI, Motor.kTimeoutMs)
        self.directionMotor.config_kD(Motor.kSlotIdx, DirectionMotor.kD, Motor.kTimeoutMs)
        self.directionMotor.config_kF(Motor.kSlotIdx, DirectionMotor.kF, Motor.kTimeoutMs)
        self.directionMotor.configMotionCruiseVelocity(DirectionMotor.kCruiseVel, Motor.kTimeoutMs)
        self.directionMotor.configMotionAcceleration(DirectionMotor.kCruiseAccel, Motor.kTimeoutMs)

        self.driveMotor = driveMotorController
        self.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Motor.kTimeoutMs)

        self.driveMotor.config_kP(Motor.kSlotIdx, DriveMotor.kP, Motor.kTimeoutMs)
        self.driveMotor.config_kI(Motor.kSlotIdx, DriveMotor.kI, Motor.kTimeoutMs)
        self.driveMotor.config_kD(Motor.kSlotIdx, DriveMotor.kD, Motor.kTimeoutMs)

        self.distancePerPulse = math.tau * Larry.kWheelSize / 4096
        self.turningPerPulse = math.tau / 4096

        self.turningEncoder = CANCoder
        self.turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Motor.kTimeoutMs)
        self.turningEncoder.configSensorDirection(True, Motor.kTimeoutMs)
        self.turningEncoder.configMagnetOffset(offset)


        