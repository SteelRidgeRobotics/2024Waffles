from enum import Enum

class DriverController:
    port = 0
    deadband = 0.15

class Larry:
    kWheelSize = 0.1 # meters
    kMaxSpeed = 3.658 # m/s
    kMaxRotRate = 10.472 # rad/s
    kDriveBaseRadius = 0.43 # meters

class MotorIDs:
    LEFT_FRONT_DRIVE = 0
    LEFT_REAR_DRIVE = 1
    RIGHT_FRONT_DRIVE = 2
    RIGHT_REAR_DRIVE = 3

    LEFT_FRONT_DIRECTION = 4
    LEFT_REAR_DIRECTION = 5
    RIGHT_FRONT_DIRECTION = 6
    RIGHT_REAR_DIRECTION = 7

class CANIDs:
    LEFT_FRONT = 10
    RIGHT_FRONT = 12
    LEFT_REAR = 11
    RIGHT_REAR = 13
    
class DriveConstants:
    
    def __init__(self, arbFF: float=0.0, F: float=0.0, P: float=0.0, I: float=0.0, IZone: float=0.0, D: float=0.0) -> None:
        self.kArbFF = arbFF
        self.kF = F
        self.kP = P
        self.kI = I
        self.kIZone = IZone
        self.kD = D
        
leftFrontDriveConstants = DriveConstants(arbFF=0.055)
rightFrontDriveConstants = DriveConstants(arbFF=0.056)
leftRearDriveConstants = DriveConstants(arbFF=0.054)
rightRearDriveConstants = DriveConstants(arbFF=0.055)

class CANOffsets:
    kLeftFrontOffset = 350.5078125
    kRightFrontOffset = 324.755859375
    kLeftRearOffset = 179.12109375
    kRightRearOffset = 28.828125

class DirectionMotor:
    kP = 0.6
    kI = 0.004
    kD = 2
    kF = 0.05282272
    kIZone = 150
    kCruiseVel = 21134.0
    kCruiseAccel = 21134.0 

class Motor:
    kTimeoutMs = 20
    kSlotIdx = 0
    kPIDLoopIdx = 0
    kVoltCompensation = 5
    kGearRatio = (150 / 7)
