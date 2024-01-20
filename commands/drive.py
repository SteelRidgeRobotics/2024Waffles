from commands2 import Command
from constants import *
from math import fabs
from subsystems.swerve import Swerve
from wpilib import SmartDashboard, XboxController
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds

class DriveByController(Command):
    transXSlew = SlewRateLimiter(10)
    transYSlew = SlewRateLimiter(10)
    rotSlew = SlewRateLimiter(10)

    def __init__(self, swerve: Swerve, controller: XboxController) -> None:
        super().__init__()

        self.swerve = swerve
        self.addRequirements(self.swerve)

        self.controller = controller

    def initialize(self) -> None:
        self.swerve.initialize()        
    
    def execute(self) -> None:
        translationX = self.controller.getLeftY()
        translationY = self.controller.getLeftX()
        rotation = self.controller.getRightX()

        translationY = self.transXSlew.calculate(deadband(translationY, DriverController.deadband) ** 3)
        translationX = self.transYSlew.calculate(deadband(translationX, DriverController.deadband) ** 3)
        rotation = self.rotSlew.calculate(deadband(rotation, DriverController.deadband) ** 3)

        self.swerve.drive(ChassisSpeeds(translationX * Larry.kMaxSpeed, translationY * Larry.kMaxSpeed, rotation * Larry.kMaxRotRate), fieldRelative=True)
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
def deadband(value: float, band: float):
    """
    value is the value we want to deadband
    the band is the abs value the value can not be less than
    """
    # this makes sure that joystick drifting is not an issue.
    # It takes the small values and forces it to be zero if smaller than the 
    # band value
    if fabs(value) <= band:
        return 0
    else:
        return value
