from commands2 import Command
from constants import Waffles
from subsystems.swerve import Swerve
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpilib import XboxController

class DebugDirectionMotors(Command):
    
    def __init__(self, swerve: Swerve, controller: XboxController):
        super().__init__()
        
        self.swerve = swerve
        self.addRequirements(self.swerve)
        
        self.controller = controller
    
    def initialize(self):
        self.currentAngle = 0
        self.isAPressed = self.isBPressed = False
    
    def execute(self):
        if not self.isAPressed and self.controller.getAButtonPressed():
            self.isAPressed = True
            self.currentAngle += 90
        else:
            self.isAPressed = False
            
        if not self.isBPressed and self.controller.getBButtonPressed():
            self.isBPressed = True
            self.currentAngle -= 90
        else:
            self.isBPressed = False
        
        state = SwerveModuleState(speed=0, angle=Rotation2d.fromDegrees(self.currentAngle))
        
        self.swerve.set_module_states((state, state, state, state), optimizeAngle=False)
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class DebugDriveMotors(Command):
    
    def __init__(self, swerve: Swerve, controller: XboxController):
        super().__init__()
        
        self.swerve = swerve
        self.addRequirements(self.swerve)
        
        self.controller = controller
        
    def initialize(self):
        self.isAPressed = self.isBPressed = False
        self.speed = 0
        
    def execute(self):
        
        if not self.isAPressed and self.controller.getAButtonPressed():
            self.isAPressed = True
            self.speed = Waffles.k_max_speed
        else:
            self.isAPressed = False
            
        if not self.isBPressed and self.controller.getBButtonPressed():
            self.isBPressed = True
            self.speed = -Waffles.k_max_speed
        else:
            self.isBPressed = False
            
        state = SwerveModuleState(speed=self.speed, angle=Rotation2d())
        self.swerve.set_module_states((state, state, state, state), optimizeAngle=False)
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
