from commands2 import Subsystem

from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.configs.cancoder_configs import AbsoluteSensorRangeValue
from phoenix6.configs.config_groups import *
from phoenix6.controls import MotionMagicVelocityTorqueCurrentFOC, MotionMagicVoltage
from phoenix6.hardware import CANcoder, TalonFX

from wpilib import RobotBase, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from constants import Constants
from conversions import *

class SwerveModule(Subsystem):
    """A single module on the drive train, comprised of 2 motors: one for driving, one for steering."""
    
    # Creating configs for each device
    
    ### Drive motor ###
    drive_config = TalonFXConfiguration()
    
    # Tuning
    drive_config.slot0.k_p = Constants.DriveConfig.k_p
    drive_config.slot0.k_i = Constants.DriveConfig.k_i
    drive_config.slot0.k_d = Constants.DriveConfig.k_d
    drive_config.slot0.k_s = Constants.DriveConfig.k_s
    drive_config.slot0.k_v = Constants.DriveConfig.k_v
    drive_config.slot0.k_a = Constants.DriveConfig.k_a
    
    # Supply current
    drive_config.current_limits.supply_current_limit_enable = Constants.DriveConfig.k_enable_supply_limit
    drive_config.current_limits.supply_current_limit = Constants.DriveConfig.k_supply_limit
    
    # Stator current
    drive_config.current_limits.stator_current_limit_enable = Constants.DriveConfig.k_enable_stator_limit
    drive_config.current_limits.stator_current_limit = Constants.DriveConfig.k_stator_limit
    
    # Motion Magic
    drive_config.motion_magic.motion_magic_cruise_velocity = Constants.DriveConfig.k_cruise_velocity
    drive_config.motion_magic.motion_magic_acceleration = Constants.DriveConfig.k_cruise_acceleration
    drive_config.motion_magic.motion_magic_jerk = Constants.DriveConfig.k_jerk
    
    # Neutral Mode
    drive_config.motor_output.neutral_mode = Constants.DriveConfig.k_neutral_mode
    
    # Feedback
    drive_config.feedback.sensor_to_mechanism_ratio = Constants.DriveConfig.k_gear_ratio
    
    
    ### Steer motor ###
    steer_config = TalonFXConfiguration()
    
    # Tuning
    steer_config.slot0.k_p = Constants.SteerConfig.k_p
    steer_config.slot0.k_i = Constants.SteerConfig.k_i
    steer_config.slot0.k_d = Constants.SteerConfig.k_d
    steer_config.slot0.k_s = Constants.SteerConfig.k_s
    steer_config.slot0.k_v = Constants.SteerConfig.k_v
    steer_config.slot0.k_a = Constants.SteerConfig.k_a
    
    # Supply current
    steer_config.current_limits.supply_current_limit_enable = Constants.SteerConfig.k_enable_supply_limit
    steer_config.current_limits.supply_current_limit = Constants.SteerConfig.k_supply_limit
    
    # Stator current
    steer_config.current_limits.stator_current_limit_enable = Constants.SteerConfig.k_enable_stator_limit
    steer_config.current_limits.stator_current_limit = Constants.SteerConfig.k_stator_limit
    
    # Motion Magic
    steer_config.motion_magic.motion_magic_cruise_velocity = Constants.SteerConfig.k_cruise_velocity
    steer_config.motion_magic.motion_magic_acceleration = Constants.SteerConfig.k_cruise_acceleration
    steer_config.motion_magic.motion_magic_jerk = Constants.SteerConfig.k_jerk
    
    # Sensors and Feedback
    steer_config.feedback.feedback_sensor_source = Constants.SteerConfig.k_remote_sensor_source
    
    steer_config.closed_loop_general.continuous_wrap = True # This does our angle optimizations for us (yay)
    
    steer_config.feedback.rotor_to_sensor_ratio = Constants.SteerConfig.k_gear_ratio
    
    
    ### Encoder ###
    encoder_config = CANcoderConfiguration()
    
    # Sensor range (basically makes our offset be between [-0.5, 0.5))
    encoder_config.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
    
    
    def __init__(self, drive_id: int, steer_id: int, encoder_id: int, encoder_offset: float) -> None:
        
        # Finish configuring
        
        # Steer motors (set sensor id)
        module_steer_config = self.steer_config # (don't edit static configs! Move them into a temp variable and edit that)
        module_steer_config.feedback.feedback_remote_sensor_id = encoder_id # Bind the encoder to the talon

        # Encoder configs (set magnet offset)
        module_encoder_config = self.encoder_config
        module_encoder_config.magnet_sensor.magnet_offset = encoder_offset
        
        
        # Create CAN devices and apply the new configs
        
        self.drive_talon = TalonFX(drive_id)
        self.drive_talon.configurator.apply(self.drive_config)
        
        self.steer_talon = TalonFX(steer_id)
        self.steer_talon.configurator.apply(module_steer_config)
        
        self.encoder = CANcoder(encoder_id)
        self.encoder.configurator.apply(module_encoder_config)
        
        # Create control requests and set them to the talons
        self.steer_request = MotionMagicVoltage(0)
        self.drive_request = MotionMagicVelocityTorqueCurrentFOC(0)
        
        self.steer_talon.set_control(self.steer_request)
        self.drive_talon.set_control(self.drive_request)
        
        # Sim states
        self.drive_sim = self.drive_talon.sim_state
        self.steer_sim = self.steer_talon.sim_state
        self.encoder_sim = self.encoder.sim_state
        
        # Remove encoder offset if we're in simulation
        self.encoder_sim.set_raw_position(-encoder_offset + 0.25)
        
        # Create variables to keep track of our modules "state"
        self.previous_desired_angle = 0
        self.speed_multiplier = 1
        
    def simulationPeriodic(self) -> None:
        # Position encoders don't update in sim, 
        # so we need to read the current velocity and get an estimate of the distance moved each tick.
        drive_displacement = self.drive_talon.get_rotor_velocity().value * 0.02
        
        # Update sim states
        self.drive_sim.add_rotor_position(drive_displacement)
        
    def get_angle(self) -> Rotation2d:
        """Returns the current angle of the wheel by converting the steer motor position into degrees."""
        
        degrees = rots_to_degs(self.steer_talon.get_position().value)
        return Rotation2d.fromDegrees(degrees)
    
    def get_speed(self) -> float:
        """Returns the module's current driving speed (m/s)."""
        
        return rot_to_meters(self.drive_talon.get_velocity().value)
        
    def get_position(self) -> SwerveModulePosition:
        """Returns the current position of the module; distance travelled and current angle."""
        
        distance = rot_to_meters(self.drive_talon.get_position().value)
        
        return SwerveModulePosition(
            distance, 
            self.get_angle()
        )
    
    def get_state(self) -> SwerveModuleState:
        """Returns the module's current state; current speed (m/s) and current angle."""
        
        return SwerveModuleState(
            self.get_speed(),
            self.get_angle()
        )
        
    def set_desired_state(self, state: SwerveModuleState) -> None:
        """Sets the motor control requests to the desired state."""
        
        desired_angle = state.angle.degrees() % 360
        
        angle_diff = desired_angle - self.previous_desired_angle
        angle_dist = math.fabs(angle_diff) # Calculate the angle distance since we last updated

        # If we have to travel more than 90 degrees CCW or CW, 
        # Invert the direction the wheel is supposed to travel in
        # (The continuous wrap handles the position for us)
        if 90 < angle_dist < 270 and RobotBase.isReal(): # Check is robot is real since continuous wrap doesn't work in sim 
            self.speed_multiplier *= -1
            
        # Update control requests
        self.steer_request.position = degs_to_rots(desired_angle)
        self.drive_request.velocity = meters_to_rots(state.speed * self.speed_multiplier)
        
        # Update sim states
        self.drive_sim.set_rotor_velocity(meters_to_rots(state.speed) * self.speed_multiplier * Constants.DriveConfig.k_gear_ratio)
        
        # Continuous wrap doesn't work in sim, so we add the difference in the angles and add it to the encoder and motor (in rots)
        # That way the wheels stay aligned and can rotate both directions
        self.steer_sim.add_rotor_position(degs_to_rots(angle_diff) * Constants.SteerConfig.k_gear_ratio)
        self.encoder_sim.add_position(degs_to_rots(angle_diff))
        
        # This current angle will be the preivious one in the next update, so update it here.        
        self.previous_desired_angle = desired_angle
        