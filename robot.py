from commands2 import *
from constants import *
from commands.drive import DriveByController
from subsystems.swerve import Swerve, SwerveModule
from wpilib import run, XboxController

class EvilLarry(TimedCommandRobot):
    driverController = XboxController(DriverController.port)

    swerve = Swerve(
        SwerveModule("LF", MotorIDs.LEFT_FRONT_DIRECTION, MotorIDs.LEFT_FRONT_DRIVE, CANIDs.LEFT_FRONT, CANOffsets.kLeftFrontOffset),
        SwerveModule("LR", MotorIDs.LEFT_REAR_DIRECTION, MotorIDs.LEFT_REAR_DRIVE, CANIDs.LEFT_REAR, CANOffsets.kLeftRearOffset),
        SwerveModule("RF", MotorIDs.RIGHT_FRONT_DIRECTION, MotorIDs.RIGHT_FRONT_DRIVE, CANIDs.RIGHT_FRONT, CANOffsets.kRightFrontOffset),
        SwerveModule("RR", MotorIDs.RIGHT_REAR_DIRECTION, MotorIDs.RIGHT_REAR_DRIVE, CANIDs.RIGHT_REAR, CANOffsets.kRightRearOffset)
    )

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def teleopPeriodic(self) -> None:
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driverController))

if __name__ == "__main__":
    run(EvilLarry)
