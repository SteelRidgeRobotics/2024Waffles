from enum import Enum

class DriverController:
    port = 0
    deadband = 0.15

class Larry:
    kWheelSize = 0.1 # meters

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
    RIGHT_FRONT = 11
    LEFT_REAR = 12
    RIGHT_REAR = 13

class COffsets:
    kFLOffset = 350.5078125
    kRLOffset = 179.12109375
    kFROffset = 324.755859375
    kRROffset = 28.828125

class DirectionMotor:
    kP = 0
    kI = 0
    kD = 0
    kF = 0
    kIZone = 0
    kCruiseVel = 21134.0
    kCruiseAccel = 21134.0 

class DriveMotor:
    kP = 0
    kI = 0
    kD = 0
    kCruiseVel = 21134.0
    kCruiseAccel = 21134.0 

class Motor:
    kTimeoutMs = 20
    kSlotIdx = 0
    kPIDLoopIdx = 0
    kVoltCompensation = 12.6
    kGearRatio = (150 / 7)
