from commands2 import *
from constants import *
from math import fabs
from subsystems.swerve import Swerve
from wpilib import XboxController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

class DriveByController(Command):

    def __init__(self, swerve: Swerve, controller: XboxController) -> None:
        super().__init__()

        self.swerve = swerve
        self.addRequirements(self.swerve)

        self.controller = controller

    def initialize(self):
        self.swerve.field_relative_drive(ChassisSpeeds())
    
    def execute(self) -> None:
        translation_x = self.controller.getLeftY()
        translation_y = self.controller.getLeftX()
        rotation = self.controller.getRightX()
        
        # Bumper Slowdown
        slowdown_mult = 1
        if self.controller.getLeftBumper():
            slowdown_mult += 0.5
        if self.controller.getRightBumper():
            slowdown_mult += 0.5

        translation_y = -deadband(translation_y, DriverController.deadband) ** 3
        translation_x = -deadband(translation_x, DriverController.deadband) ** 3
        rotation = -deadband(rotation, DriverController.deadband) ** 3
        
        self.swerve.field_relative_drive(ChassisSpeeds(translation_x * Waffles.k_max_module_speed / slowdown_mult, 
                                        translation_y * Waffles.k_max_module_speed / slowdown_mult, 
                                        rotation * Waffles.k_max_rot_rate / slowdown_mult))
    
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
