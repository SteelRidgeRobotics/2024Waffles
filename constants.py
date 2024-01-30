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
    
class DriveMotor:
    karbFF = 0.054

class CANOffsets:
    kLeftFrontOffset = 0
    kRightFrontOffset = 324.755859375
    kLeftRearOffset = 179.12109375
    kRightRearOffset = 28.828125

class DirectionMotor:
    k_p = 0.8
    #k_p = 0.120117
    # k_i = 0.800782
    k_i = 0
    k_d = 0.0004
    k_v = 0.2
    #k_v = 0.105748
    kCruiseVel = 103.193
    kCruiseAccel = 103.193
    kJerk = 1600

    k_s_LeftFront = 0.25
    k_s_LeftRear = 0.25
    k_s_RightFront = 0.25
    k_s_RightRear = 0.25

class Motor:
    kTimeoutMs = 20
    kSlotIdx = 0
    kPIDLoopIdx = 0
    kVoltCompensation = 5
    kGearRatio = (150 / 7)
