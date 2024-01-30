from commands2 import *
from constants import *
from commands.debug import *
from commands.drive import DriveByController
from pathplannerlib.auto import PathPlannerAuto
from subsystems.swerve import Swerve
from wpilib import SendableChooser, SmartDashboard, XboxController

class Waffles(TimedCommandRobot):
    driver_controller = XboxController(DriverController.port)

    auto_chooser = SendableChooser()
    
    def __init__(self, period = 0.02) -> None:
        super().__init__(period)
        
    def robotInit(self) -> None:
        super().robotInit()
        
        self.swerve: Swerve = Swerve()
        
        self.auto_chooser.setDefaultOption("Failsafe (F2M)", PathPlannerAuto("FAILSAFE"))
        self.auto_chooser.addOption("B2M", PathPlannerAuto("Backward 2 Meters"))
        self.auto_chooser.addOption("R2M", PathPlannerAuto("Right 2 Meters"))
        self.auto_chooser.addOption("L2M", PathPlannerAuto("Left 2 Meters"))
        
        SmartDashboard.putData("Auto Route", self.auto_chooser)

    def getSelectedAutoCommand(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()

    def autonomousInit(self) -> None:
        self.getSelectedAutoCommand().schedule()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def teleopPeriodic(self) -> None:
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driver_controller))
        #self.swerve.setDefaultCommand(DebugDirectionMotors(self.swerve, self.driverController))
        #self.swerve.setDefaultCommand(DebugDriveMotors(self.swerve, self.driverController))
