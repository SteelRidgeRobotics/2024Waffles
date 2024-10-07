from commands2.button import JoystickButton

from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.path import PathConstraints

from wpilib import SendableChooser, XboxController
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from constants import Constants
from subsystems.drivetrain import Drivetrain

from commands.autoAlign import AutoAlign


class RobotContainer:
    # Controllers
    driver_controller = XboxController(Constants.Controller.k_driver_controller_port)
    
    # Subsystems
    drivetrain = Drivetrain()
    
    # Auto Chooser
    auto_chooser = SendableChooser()
    auto_chooser.setDefaultOption("Do Nothing :(", None)
    auto_chooser.addOption("Under The Stage", PathPlannerAuto("UnderTheStage"))
    auto_chooser.addOption("Move Forward", PathPlannerAuto("MoveForward"))
    auto_chooser.addOption("Under, Around, and Forward", PathPlannerAuto("UnderAroundAndForward"))
    auto_chooser.addOption("Speaker To Speaker", PathPlannerAuto("SpeakerToSpeaker"))
    
    # Send to Shuffleboard
    Shuffleboard.getTab("Main").add("Auto Selector", auto_chooser).withWidget(BuiltInWidgets.kComboBoxChooser)
    
    
    def __init__(self) -> None:
        
        # Create both commands for the drivetrain, one for robot-centric, one for field-relative
        # (both snazzy lambdas)
        self.robot_centric_command = self.drivetrain.runOnce(
            lambda: self.drivetrain.drive_robot_centric(
                ChassisSpeeds(
                   -self.driver_controller.getLeftY() * Constants.Drivetrain.k_max_attainable_speed, # Speed forward and backward 
                   -self.driver_controller.getLeftX() * Constants.Drivetrain.k_max_attainable_speed, # Speed Left and right
                   -self.driver_controller.getRightX() * Constants.Drivetrain.k_max_rot_rate # Rotation speed
                ) 
            )
        ).repeatedly() # Tells it to run the command forever until we tell it not to.
        
        self.field_relative_command = self.drivetrain.runOnce(
            lambda: self.drivetrain.drive_field_relative(
                ChassisSpeeds(
                   -self.driver_controller.getLeftY() * Constants.Drivetrain.k_max_attainable_speed, # Speed forward and backward 
                   -self.driver_controller.getLeftX() * Constants.Drivetrain.k_max_attainable_speed, # Speed Left and right
                   -self.driver_controller.getRightX() * Constants.Drivetrain.k_max_rot_rate # Rotation speed
                ) 
            )
        ).repeatedly()
        
        # Field-relative by default
        self.drivetrain.setDefaultCommand(AutoAlign(self.drivetrain))

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
    