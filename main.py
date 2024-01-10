from commands2 import TimedCommandRobot
from constants import *
from wpilib import run, XboxController

class EvilLarry(TimedCommandRobot):
    driverController = XboxController(DriverController.port)

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)

if __name__ == "__main__":
    run(EvilLarry)
