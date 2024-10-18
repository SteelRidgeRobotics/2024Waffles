from commands2.button import JoystickButton

import math

from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.path import PathConstraints

from wpilib import SendableChooser, XboxController
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from constants import Constants
from subsystems.drivetrain import Drivetrain

import math

class RobotContainer:
    # Controllers
    driver_controller = XboxController(Constants.Controller.k_driver_controller_port)
    
    # Subsystems
    drivetrain = Drivetrain()
    
    # Auto Chooser
    auto_chooser = SendableChooser()
    auto_chooser.setDefaultOption("Speaker To Speaker", PathPlannerAuto("SpeakerToSpeaker"))
    auto_chooser.addOption("Do Nothing :(", None)
    auto_chooser.addOption("Under The Stage", PathPlannerAuto("UnderTheStage"))
    auto_chooser.addOption("Move Forward", PathPlannerAuto("MoveForward"))
    auto_chooser.addOption("Under, Around, and Forward", PathPlannerAuto("UnderAroundAndForward"))
    auto_chooser.addOption("4 Amp Auto", PathPlannerAuto("4AmpAuto"))
    
    # Send to Shuffleboard
    Shuffleboard.getTab("Main").add("Auto Selector", auto_chooser).withWidget(BuiltInWidgets.kComboBoxChooser)
    
    
    def __init__(self) -> None:

        # Create both commands for the drivetrain, one for robot-centric, one for field-relative
        # (both snazzy lambdas)
        self.robot_centric_command = self.drivetrain.runOnce(
            lambda: self.drivetrain.drive_robot_centric(
                ChassisSpeeds(
                   -deadband(self.driver_controller.getLeftY(), Constants.Controller.k_left_deadband_y) * Constants.Drivetrain.k_max_attainable_speed, # Speed forward and backward

                   -deadband(self.driver_controller.getLeftX(), Constants.Controller.k_left_deadband_x) * Constants.Drivetrain.k_max_attainable_speed, # Speed Left and right

                   -deadband(self.driver_controller.getRightX(), Constants.Controller.k_right_deadband_x) * Constants.Drivetrain.k_max_rot_rate # Rotation speed
                )
            )
        ).repeatedly() # Tells it to run the command forever until we tell it not to.
        
        if Constants.Controller.k_fully_field_relative:
            self.field_relative_command = self.drivetrain.runOnce(
                lambda: self.drivetrain.drive_fully_field_relative(
                    ChassisSpeeds(
                        -deadband(self.driver_controller.getLeftY(), Constants.Controller.k_left_deadband_y) * Constants.Drivetrain.k_max_attainable_speed,

                        -deadband(self.driver_controller.getLeftX(), Constants.Controller.k_left_deadband_x) * Constants.Drivetrain.k_max_attainable_speed,

                        (self.driver_controller.getLeftTriggerAxis() - self.driver_controller.getRightTriggerAxis()) * Constants.Drivetrain.k_max_rot_rate # If your triggers are starting to drift somehow, get a new controller :skull:
                    ),
                    get_desired_angle(
                        deadband(self.driver_controller.getRightY(), Constants.Controller.k_right_deadband_y), 
                        
                        -deadband(self.driver_controller.getRightX(), Constants.Controller.k_right_deadband_x), 

                        Rotation2d.fromDegrees(-self.driver_controller.getPOV()) if self.driver_controller.getPOV() != -1 else None
                    )
                )
            ).repeatedly()

        else:
            self.field_relative_command = self.drivetrain.runOnce(
                lambda: self.drivetrain.drive_field_relative(
                    ChassisSpeeds(
                        -deadband(self.driver_controller.getLeftY(), Constants.Controller.k_left_deadband_x) * Constants.Drivetrain.k_max_attainable_speed, # Speed forward and backward 

                        -deadband(self.driver_controller.getLeftX(), Constants.Controller.k_left_deadband_y) * Constants.Drivetrain.k_max_attainable_speed, # Speed Left and right

                        -deadband(self.driver_controller.getRightX(), Constants.Controller.k_right_deadband_x) * Constants.Drivetrain.k_max_rot_rate # Rotation speed
                    ) 
                )
            ).repeatedly()
        
        # Field-relative by default
        self.drivetrain.setDefaultCommand(self.field_relative_command)
        
        self.configure_button_bindings()
        
    def get_selected_auto(self) -> PathPlannerAuto | None:
        """Returns the selected auto."""
        
        return self.auto_chooser.getSelected()
    
    def configure_button_bindings(self) -> None:
        """Setup all button bindings."""
        
        # Switch to field-relative
        JoystickButton(self.driver_controller, XboxController.Button.kLeftBumper).onTrue(self.field_relative_command)
        
        # Switch to robot-centric
        JoystickButton(self.driver_controller, XboxController.Button.kRightBumper).onTrue(self.robot_centric_command)

        # Drive to amp
        JoystickButton(self.driver_controller, XboxController.Button.kA).whileTrue(
            self.drivetrain.pathfind_to_path(
                "AlignToAmp", 
                path_constraints=PathConstraints(4.5, 4.5, 3.14, 3.14),
                rotation_delay_distance=5
            )
        )

@staticmethod
def get_desired_angle(y: float, x: float, fallback: Rotation2d = None) -> Rotation2d | None:
    """For fully-field relative drive. Converts from joystick axes into Rotation2d.

    Args:
        y (float): Joystick Y axis. For Xbox Controllers, this value is inverted.
        x (float): Joystick X axis. For Xbox Controllers, this value is inverted.
        fallback (Rotation2d, optional): Rotation2d to return if y and x aren't at a high enough magnitude, useful if wanting to use the POV wheel. Defaults to None.

    Returns:
        Rotation2d | None: The converted Rotation2d if the joystick is "under use", otherwise returns the fallback.
    """
    if y**2 + x**2 <= math.sqrt(2)/2:
        return fallback
    return Rotation2d(math.atan2(y, x) + (math.pi / 2))

@staticmethod
def deadband(requested_value: float, deadband: float) -> float:
    return requested_value if abs(requested_value) > deadband or not Constants.Controller.k_deadband_enabled else 0
