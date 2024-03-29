from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue

class DriverController:
    port = 0
    deadband = 0.15

class Waffles:
    k_wheel_size = 0.1 # meters
    k_max_module_speed =  4.654 # m/s
    k_max_rot_rate = 10.472 # rad/s
    k_drive_base_radius = 0.43 # meters
    
class DriveMotorConstants:
    kS = 0.4
    kV = 0
    kA = 0
    kP = 0.133
    kI = 0
    kD = 0
    kInvert = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    kNeutral = NeutralModeValue.BRAKE
    kRatio = 27 / 4
    
class DirectionMotorConstants:
    kS = 0.26
    kV = 0
    kA = 0
    kP = 12
    kI = 10
    kD = 0
    kNeutral = NeutralModeValue.BRAKE
    kInvert = InvertedValue.CLOCKWISE_POSITIVE
    kRatio = 150 / 7
        
class MotorIDs:
    LEFT_FRONT_DRIVE = 1
    LEFT_REAR_DRIVE = 2
    RIGHT_FRONT_DRIVE = 3
    RIGHT_REAR_DRIVE = 4

    LEFT_FRONT_DIRECTION = 5
    LEFT_REAR_DIRECTION = 6
    RIGHT_FRONT_DIRECTION = 7
    RIGHT_REAR_DIRECTION = 8

class CANIDs:
    LEFT_FRONT = 5
    LEFT_REAR = 6
    RIGHT_FRONT = 7
    RIGHT_REAR = 8
