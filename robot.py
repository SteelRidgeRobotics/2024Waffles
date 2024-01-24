from commands2 import *
from constants import *
from commands.debug import *
from commands.drive import DriveByController
from pathplannerlib.auto import PathPlannerAuto
from subsystems.swerve import Swerve, SwerveModule
from wpilib import SendableChooser, SmartDashboard, XboxController

class Waffles(TimedCommandRobot):
    driverController = XboxController(DriverController.port)

    autoChooser = SendableChooser()
    
    swerve = Swerve(
        SwerveModule("LF", MotorIDs.LEFT_FRONT_DIRECTION, MotorIDs.LEFT_FRONT_DRIVE, CANIDs.LEFT_FRONT, CANOffsets.kLeftFrontOffset),
        SwerveModule("LR", MotorIDs.LEFT_REAR_DIRECTION, MotorIDs.LEFT_REAR_DRIVE, CANIDs.LEFT_REAR, CANOffsets.kLeftRearOffset),
        SwerveModule("RF", MotorIDs.RIGHT_FRONT_DIRECTION, MotorIDs.RIGHT_FRONT_DRIVE, CANIDs.RIGHT_FRONT, CANOffsets.kRightFrontOffset),
        SwerveModule("RR", MotorIDs.RIGHT_REAR_DIRECTION, MotorIDs.RIGHT_REAR_DRIVE, CANIDs.RIGHT_REAR, CANOffsets.kRightRearOffset)
    )

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)
        
    def robotInit(self) -> None:
        super().robotInit()
        
        self.autoChooser.setDefaultOption("Failsafe (F2M)", PathPlannerAuto("FAILSAFE"))
        self.autoChooser.addOption("B2M", PathPlannerAuto("Backward 2 Meters"))
        self.autoChooser.addOption("R2M", PathPlannerAuto("Right 2 Meters"))
        self.autoChooser.addOption("L2M", PathPlannerAuto("Left 2 Meters"))
        
        SmartDashboard.putData("Auto Route", self.autoChooser)

    def getSelectedAutoCommand(self) -> PathPlannerAuto:
        return self.autoChooser.getSelected()

    def autonomousInit(self) -> None:
        self.getSelectedAutoCommand().schedule()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def teleopPeriodic(self) -> None:
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driverController))
        #self.swerve.setDefaultCommand(DebugDirectionMotors(self.swerve, self.driverController))
        #self.swerve.setDefaultCommand(DebugDriveMotors(self.swerve, self.driverController))
