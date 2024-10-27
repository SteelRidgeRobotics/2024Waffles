from phoenix6.configs.talon_fx_configs import *
from wpilib import RobotBase
from wpimath.geometry import Translation2d

from math import pi


class Constants:
    
    class Controller:
        k_driver_controller_port = 0

        k_fully_field_relative = True

        k_deadband_enabled = True # Enable to tweak values if testing with personal controllers.

        k_left_deadband_x = 0.1
        k_left_deadband_y = 0.1
        k_right_deadband_x = 0.1
        k_right_deadband_y = 0.1
    
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
        k_sim_wheel_size = (0.25, 0.25) # Size of the wheel poses on the Field2d widget (length, width) (both in meters)

        k_wheel_size = 0.1 # Diameter of wheels in meters
        k_drive_base_radius = 0.83 # Radius from center of robot to swerve modules in meters
        
        k_max_attainable_speed = 4.654 # Max speed of modules in m/s
        k_max_rot_rate = 5.607 # Max chassis rotation rate (rad/s)
        k_max_rot_acceleration = 1 * pi
        k_max_rot_deceleration = 1 * pi

        # These are used for the PID controller for fully field-relative rotating.
        k_turn_p = 10
        k_turn_d = 0.4 # 10-4 REFERENCE!!!
        k_angle_tolerance = 5.0

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
        k_module_locations = (
            Translation2d(0.587, 0.587), # Left front
            Translation2d(-0.587, 0.587), # Left rear
            Translation2d(0.587, -0.587), # Right front
            Translation2d(-0.587, -0.587) # Right rear
        )
    
    class DriveConfig:
        # PID and Feedforward
        k_p = 25
        k_i = 0
        k_d = 0
        
        k_s = 14.9
        k_v = 0.12
        k_a = 0
        
        k_neutral_mode = NeutralModeValue.BRAKE
        
        # Torque limits
        k_torque_neutral_deadband = 0 # Default: 0A
        k_peak_forward_torque_current = 800 # Default: 800A
        k_peak_reverse_torque_current = -800 # Default: -800A
        
        # Current limits (only needed if using non-torque control)
        k_enable_supply_limit = False
        k_supply_limit = 0
        
        k_enable_stator_limit = False
        k_stator_limit = 0
        
        # Motion Magic
        k_cruise_velocity = 14
        k_cruise_acceleration = 52
        k_jerk = 0
        
        # Physical Properties
        k_gear_ratio = 27 / 4
        
    class SteerConfig:
        # PID and Feedforward
        k_p = 250
        k_i = 0
        k_d = 40
        
        k_s = 4.6
        k_v = 0.12
        k_a = 0.8
        
        k_neutral_mode = NeutralModeValue.BRAKE
        
        # Torque limits
        k_torque_neutral_deadband = 0 # Default: 0A
        k_peak_forward_torque_current = 800 # Default: 800A
        k_peak_reverse_torque_current = -800 # Default: -800A
        
        # Current limits (only needed if using non-torque control)
        k_enable_supply_limit = False
        k_supply_limit = 0
        
        k_enable_stator_limit = False
        k_stator_limit = 0
        
        # Motion Magic
        k_cruise_velocity = 4
        k_cruise_acceleration = 8
        k_jerk = 75
        
        # Sensors (in case we switch it to a synced cancoder later on)
        k_remote_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        
        # Physical Properties
        k_gear_ratio = 150 / 7
        
    class PathPlanner:
        ## PID Constants ##
        
        # Translation
        k_translation_p = 5
        k_translation_i = 0
        k_translation_d = 0
        k_translation_i_zone = 0 # This basically means "how powerful can k_i be at any given time"
        
        # Rotation
        k_rotation_p = 5
        k_rotation_i = 0
        k_rotation_d = 0
        k_rotation_i_zone = 0
        
    class Limelight:
        
        k_enable_vision_odometry = RobotBase.isReal() # False if there's no Limelight on the robot.
        
        k_limelight_name = "limelight" # "limelight" by default. Name of the limelight to use for vision.

        k_use_mega_tag_2 = True # If False, uses MegaTag 1.
        
        k_standard_deviations = [0.3, 0.3, 99999] # (x, y, radians) Basically how confident we are with our vision, lower = more confident. Angle is set really high because we have a gyro.
