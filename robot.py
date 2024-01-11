from commands2 import TimedCommandRobot
from constants import *
from subsystems.swerve import Swerve
from wpilib import run, XboxController

class EvilLarry(TimedCommandRobot):
    driverController = XboxController(DriverController.port)

    swerve = Swerve()

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)

if __name__ == "__main__":
    run(EvilLarry)
