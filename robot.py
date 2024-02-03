from commands2 import *
from constants import *
from commands.drive import DriveByController
from pathplannerlib.auto import PathPlannerAuto
from subsystems.swerve import Swerve
from wpimath.geometry import Pose2d
from wpilib import SendableChooser, SmartDashboard, XboxController
from wpilib import DriverStation

class Waffles(TimedCommandRobot):
    driver_controller = XboxController(DriverController.port)

    auto_chooser = SendableChooser()
    start_chooser = SendableChooser()
    
    def __init__(self, period = 0.02) -> None:
        super().__init__(period)
        
    def robotInit(self) -> None:
        super().robotInit()
        
        self.swerve: Swerve = Swerve()
        
        self.auto_chooser.setDefaultOption("Failsafe (F2M)", PathPlannerAuto("FAILSAFE"))
        self.auto_chooser.addOption("B2M", PathPlannerAuto("Backward 2 Meters"))
        self.auto_chooser.addOption("R2M", PathPlannerAuto("Right 2 Meters"))
        self.auto_chooser.addOption("L2M", PathPlannerAuto("Left 2 Meters"))
        
        # Start Chooser
        # This is in charge of choosing our start position. failure to change this will result in the bot doing a LOT of weird stuff.
        self.start_chooser.setDefaultOption("Failsafe (0, 0)", Pose2d())
        self.start_chooser.addOption("Blue 1", Pose2d())
        self.start_chooser.addOption("Blue 2", Pose2d())
        self.start_chooser.addOption("Blue 3", Pose2d())
        self.start_chooser.addOption("Red 1", Pose2d())
        self.start_chooser.addOption("Red 2", Pose2d())
        self.start_chooser.addOption("Red 3", Pose2d())
        
        self.start_chooser.onChange(lambda: self.swerve.reset_odometry(self.start_chooser.getSelected()))
        
        SmartDashboard.putData("Auto Route", self.auto_chooser)
        SmartDashboard.putData("Starting Position", self.start_chooser)

    def getSelectedAutoCommand(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()

    def autonomousInit(self) -> None:
        self.swerve.initialize().reset_yaw().reset_odometry(self.start_chooser.getSelected())
        self.getSelectedAutoCommand().schedule()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def teleopPeriodic(self) -> None:
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driver_controller))
