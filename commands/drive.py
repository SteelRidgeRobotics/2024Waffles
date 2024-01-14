from commands2 import CommandBase
from constants import *
from frc6343.controller.deadband import deadband
from subsystems.swerve import Swerve
from wpilib import SmartDashboard, XboxController
from wpimath.kinematics import ChassisSpeeds

class DriveByController(CommandBase):

    def __init__(self, swerve: Swerve, controller: XboxController) -> None:
        super().__init__()

        self.swerve = swerve
        self.addRequirements(self.swerve)

        self.controller = controller

    def initialize(self) -> None:
        self.swerve.initialize()
    
    def execute(self) -> None:
        translationX = -self.controller.getLeftY()
        translationY = -self.controller.getLeftX()
        rotation = -self.controller.getRightX()

        translationY = deadband(translationY, DriverController.deadband)
        translationX = deadband(translationX, DriverController.deadband)
        rotation = deadband(rotation, DriverController.deadband)

        self.swerve.drive(ChassisSpeeds(translationX * Larry.kMaxSpeed, translationY * Larry.kMaxSpeed, rotation * Larry.kMaxRotRate), fieldRelative=True)
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
