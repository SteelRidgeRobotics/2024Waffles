from phoenix6.configs.talon_fx_configs import *
from wpimath.geometry import Translation2d


class Constants:
    
    class Controller:
        k_driver_controller_port = 0
    
    class CanIDs:
        # TalonFXs
        k_left_front_drive = 1
        k_left_rear_drive = 2
        k_right_front_drive = 3
        k_right_rear_drive = 4
        
        k_left_front_direction = 5
        k_left_rear_direction = 6
        k_right_front_direction = 7
        k_right_rear_direction = 8
        
        # CANcoders
        k_left_front_encoder = 5
        k_left_rear_encoder = 6
        k_right_front_encoder = 7
        k_right_rear_encoder = 8
        
    class CanOffsets:
        k_left_front_offset = 0.474853515625
        k_left_rear_offset = -0.0009765625
        k_right_front_offset = 0.398681640625
        k_right_rear_offset = -0.41845703125
        
    class Drivetrain:
        k_wheel_size = 0.1 # Diameter of wheels in meters
        k_drive_base_radius = 0.83 # Radius from center of robot to swerve modules in meters
        
        k_max_attainable_speed = 4.654 # Max speed of modules in m/s
        k_max_rot_rate = 10.472 # Max chassis rotation rate (rad/s)
        
        class ModuleLocations:
            # To find Translation2d amount:
            # a^2 + b^2 = c^2
            # x^2 + y^2 = k_drive_base_radius
            # 2x^2 = 0.83^2 (square drivetrain, so x = y)
            # 2x^2 = 0.6889
            # x^2 = 0.34445
            # x = sqrt(0.3445) ~= 0.587
            # x ~= 0.587
            
            # +x is the front of the robot
            # +y is the left of the robot
            
            k_left_front_location = Translation2d(0.587, 0.587)
            k_left_rear_location = Translation2d(-0.587, 0.587)
            k_right_front_location = Translation2d(0.587, -0.587)
            k_right_rear_location = Translation2d(-0.587, -0.587)
    
    class DriveConfig:
        # PID and Feedforward
        k_p = 1.2
        k_i = 0
        k_d = 0
        
        k_s = 2
        k_v = 0
        k_a = 0
        
        k_neutral_mode = NeutralModeValue.BRAKE
        
        # Current limits
        k_enable_supply_limit = False
        k_supply_limit = 0
        
        k_enable_stator_limit = False
        k_stator_limit = 0
        
        # Motion Magic
        k_cruise_velocity = 75
        k_cruise_acceleration = 100
        k_jerk = 0
        
        # Physical Properties
        k_gear_ratio = 27 / 4
        
    class SteerConfig:
        # PID and Feedforward
        k_p = 12
        k_i = 10
        k_d = 0
        
        k_s = 0.26
        k_v = 0
        k_a = 0
        
        k_neutral_mode = NeutralModeValue.BRAKE
        
        # Current limits
        k_enable_supply_limit = False
        k_supply_limit = 0
        
        k_enable_stator_limit = False
        k_stator_limit = 0
        
        # Motion Magic
        k_cruise_velocity = 75
        k_cruise_acceleration = 100
        k_jerk = 0
        
        # Sensors (in case we switch it to a synced cancoder later on)
        k_remote_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        
        # Physical Properties
        k_gear_ratio = 150 / 7
        