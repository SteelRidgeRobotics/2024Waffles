from commands2.button import JoystickButton

import math

from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.path import PathConstraints

from wpilib import SendableChooser, XboxController
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.geometry import Rotation2d

from commands.drive_maintain_heading import DriveMaintainHeadingCommand
from constants import Constants
from subsystems.drivetrain import Drivetrain

import math

class RobotContainer:

    # Controllers
    driver_controller = XboxController(Constants.Controller.k_driver_controller_port)
    
    def __init__(self) -> None:

        self.drivetrain = Drivetrain()

        # Auto Chooser
        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("Speaker To Speaker", PathPlannerAuto("SpeakerToSpeaker"))
        self.auto_chooser.addOption("Do Nothing :(", None)
        self.auto_chooser.addOption("Under The Stage", PathPlannerAuto("UnderTheStage"))
        self.auto_chooser.addOption("Move Forward", PathPlannerAuto("MoveForward"))
        self.auto_chooser.addOption("Under, Around, and Forward", PathPlannerAuto("UnderAroundAndForward"))
        self.auto_chooser.addOption("4 Amp Auto", PathPlannerAuto("4AmpAuto"))
        Shuffleboard.getTab("Main").add("Auto Selector", self.auto_chooser).withWidget(BuiltInWidgets.kComboBoxChooser)

        self.drivetrain.setDefaultCommand(
            DriveMaintainHeadingCommand(self.drivetrain,
            lambda: -self.driver_controller.getLeftY(),
            lambda: -self.driver_controller.getLeftX(),
            lambda: -self.driver_controller.getRightX(),
            )
        )
        
        self.configure_button_bindings()
        
    def get_selected_auto(self) -> PathPlannerAuto | None:
        """Returns the selected auto."""
        
        return self.auto_chooser.getSelected()
    
    def configure_button_bindings(self) -> None:
        """Setup all button bindings."""

        # Drive to amp
        JoystickButton(self.driver_controller, XboxController.Button.kA).whileTrue(
            self.drivetrain.pathfind_to_path(
                "AlignToAmp", 
                path_constraints=PathConstraints(4.5, 4.5, 3.14, 3.14),
                rotation_delay_distance=5
            )
        )
