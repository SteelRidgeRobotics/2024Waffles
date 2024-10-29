from commands2.button import JoystickButton

import math

from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.commands import PathfindThenFollowPathHolonomic
from pathplannerlib.path import PathConstraints, PathPlannerPath

from wpilib import DriverStation, SendableChooser, XboxController
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard

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

        JoystickButton(self.driver_controller, XboxController.Button.kA).whileTrue(
            PathfindThenFollowPathHolonomic(
                PathPlannerPath.fromPathFile("AlignToAmp"), # The path to pathfind to
                PathConstraints(4.5, 4.5, 3.14, 3.14), # Constraints while pathfinding (max velocity, max rotation, etc)
                self.drivetrain.odometry.getEstimatedPosition, # Pose supplier
                self.drivetrain.get_robot_speed, # Speed Supplier
                lambda speeds: self.drivetrain.drive(speeds.vx, speeds.vy, speeds.omega, is_field_relative=False), # Speed Consumer
                self.drivetrain.path_follower_config,
                lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # "hey caden when do we invert the path"
                self.drivetrain, # We ARE a DRIVE-TEAM!!!!!!
                5 # Amount the robot needs to move before it rotates (when pathfinding, in meters)
            )
        )
