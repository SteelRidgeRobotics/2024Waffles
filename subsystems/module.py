from commands2 import Subsystem

from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.configs.cancoder_configs import AbsoluteSensorRangeValue
from phoenix6.configs.config_groups import *
from phoenix6.controls import MotionMagicVelocityTorqueCurrentFOC, MotionMagicTorqueCurrentFOC
from phoenix6.hardware import CANcoder, ParentDevice, TalonFX
from phoenix6.status_signal import BaseStatusSignal

from wpilib import RobotBase
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
    
    # Torque current
    drive_config.torque_current.torque_neutral_deadband = Constants.DriveConfig.k_torque_neutral_deadband
    drive_config.torque_current.peak_forward_torque_current = Constants.DriveConfig.k_peak_forward_torque_current
    drive_config.torque_current.peak_reverse_torque_current = Constants.DriveConfig.k_peak_reverse_torque_current
    
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
    drive_config.motor_output.neutral_mode = NeutralModeValue.COAST
    
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
    
    # Torque current
    steer_config.torque_current.torque_neutral_deadband = Constants.SteerConfig.k_torque_neutral_deadband
    steer_config.torque_current.peak_forward_torque_current = Constants.SteerConfig.k_peak_forward_torque_current
    steer_config.torque_current.peak_reverse_torque_current = Constants.SteerConfig.k_peak_reverse_torque_current
    
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
    steer_config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER

    steer_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
    steer_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
    
    steer_config.closed_loop_general.continuous_wrap = True # This does our angle optimizations for us (yay)
    
    # With a fused CANcoder, we change the rotor_to_sensor ratio, that way the encoder position tells us the rotations of the wheel itself
    # (Obviously this will vary from mechanism, but most of the time, if it uses a CANcoder, change this instead)
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
        if RobotBase.isReal():
            module_encoder_config.magnet_sensor.magnet_offset = encoder_offset
        else:
            module_encoder_config.magnet_sensor.magnet_offset = 0.25
        
        
        # Create CAN devices and apply the new configs
        
        self.drive_talon = TalonFX(drive_id)
        self.drive_talon.configurator.apply(self.drive_config)
        
        self.steer_talon = TalonFX(steer_id)
        self.steer_talon.configurator.apply(module_steer_config)
        
        self.encoder = CANcoder(encoder_id)
        self.encoder.configurator.apply(module_encoder_config)
        
        # Create control requests and set them to the talons
        self.steer_request = MotionMagicTorqueCurrentFOC(0)
        self.drive_request = MotionMagicVelocityTorqueCurrentFOC(0)
        
        self.steer_talon.set_control(self.steer_request)
        self.drive_talon.set_control(self.drive_request)
        
        # Sim states
        self.drive_sim = self.drive_talon.sim_state
        self.steer_sim = self.steer_talon.sim_state
        self.encoder_sim = self.encoder.sim_state

        # Set Supply Voltages
        self.drive_sim.set_supply_voltage(12)
        self.steer_sim.set_supply_voltage(12)
        self.encoder_sim.set_supply_voltage(12)
        
        ## Set StatusSignal update frequencies ##
        # Drive Motor
        self.drive_talon.get_acceleration().set_update_frequency(200)
        self.drive_talon.get_position().set_update_frequency(200)
        self.drive_talon.get_velocity().set_update_frequency(200)
        
        # Steer Motor
        self.steer_talon.get_acceleration().set_update_frequency(200)
        self.steer_talon.get_position().set_update_frequency(200)
        self.steer_talon.get_velocity().set_update_frequency(200)
        
        # CANcoder
        self.encoder.get_position().set_update_frequency(200)
        self.encoder.get_velocity().set_update_frequency(200)
        
        # Disable all unset status signals for all devices in this module
        ParentDevice.optimize_bus_utilization_for_all(self.drive_talon, self.steer_talon, self.encoder)

        if RobotBase.isReal():
            self.desired_state = self.get_state()
        else:
            from random import randint
            random_state = SwerveModuleState(angle=Rotation2d.fromDegrees(randint(0, 359)))

            self.desired_state = random_state
        
        self.previous_desired_angle = self.desired_state.angle

        self.set_desired_state(self.desired_state)
        
    def simulationPeriodic(self) -> None:
        # Position encoders don't update in sim, 
        # so we need to read the current velocity and get an estimate of the distance moved each tick.
        drive_displacement = self.drive_talon.get_rotor_velocity().value * 0.02
        
        # Update sim states
        self.drive_sim.add_rotor_position(drive_displacement)
        
    def get_angle(self) -> Rotation2d:
        """Returns the current angle of the wheel by converting the steer motor position into degrees."""

        compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self.steer_talon.get_position().refresh(), # This function doesn't refresh the status signals, so we do it here
            self.steer_talon.get_velocity().refresh(), # Same for this signal
            0.02
        )

        return Rotation2d.fromDegrees(rots_to_degs(compensated_position))
    
    def get_speed(self) -> float:
        """Returns the module's current driving speed (m/s)."""
        
        # Compensate for acceleration
        compensated_speed = BaseStatusSignal.get_latency_compensated_value(
            self.drive_talon.get_velocity().refresh(),
            self.drive_talon.get_acceleration().refresh(),
            0.06
        )
        
        return rot_to_meters(compensated_speed)
        
    def get_position(self) -> SwerveModulePosition:
        """Returns the current position of the module; distance traveled and current angle."""
        
        # Compensate for velocity
        compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self.drive_talon.get_position().refresh(),
            self.drive_talon.get_velocity().refresh()
        )
        
        distance = rot_to_meters(compensated_position)
        
        return SwerveModulePosition(
            distance, 
            self.get_angle()
        )
    
    def get_state(self) -> SwerveModuleState:
        """Returns the module's current state; current speed (m/s) and current angle."""

        return SwerveModuleState(self.get_speed(), self.get_angle())
    
    def get_target(self) -> SwerveModuleState:
        """Returns the module's desired state."""

        return self.desired_state
        
    def set_desired_state(self, state: SwerveModuleState) -> None:
        """Sets the motor control requests to the desired state."""

        # Angle optimizations and reverse velocity if needed
        state = SwerveModuleState.optimize(state, self.previous_desired_angle)
        self.desired_state = state

        self.steer_talon.set_control(
            self.steer_request.with_position(degs_to_rots(state.angle.degrees()))
        )
        
        self.drive_talon.set_control(
            self.drive_request.with_velocity(meters_to_rots(state.speed))
        )
        
        # Update sim states
        self.drive_sim.set_rotor_velocity(meters_to_rots(state.speed) * Constants.DriveConfig.k_gear_ratio)
        
        self.steer_sim.set_raw_rotor_position(degs_to_rots(state.angle.degrees()) * Constants.SteerConfig.k_gear_ratio)
        self.encoder_sim.set_raw_position(degs_to_rots(state.angle.degrees()))

        self.previous_desired_angle = state.angle

    def stop(self) -> None:
        """Stops the module from moving."""

        self.set_desired_state(
            SwerveModuleState(0, self.previous_desired_angle)
        )
        