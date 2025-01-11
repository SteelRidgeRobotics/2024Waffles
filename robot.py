from commands2 import CommandScheduler, TimedCommandRobot
from wpilib import DataLogManager, DriverStation, RobotBase

from robot_container import RobotContainer


class Waffles(TimedCommandRobot):

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)

        self.container = RobotContainer()

        DriverStation.silenceJoystickConnectionWarning(True)

        if RobotBase.isReal():
            DataLogManager.start("/home/lvuser/logs")
        else:
            DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        DataLogManager.log("robot initialized")

    # Most of these are all here to suppress warnings
    def robotPeriodic(self) -> None:
        pass
    
    def _simulationPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        DataLogManager.log("Autonomous period started")

        selected_auto = self.container.get_selected_auto()
        if selected_auto is not None:
            selected_auto.schedule()
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        DataLogManager.log("Autonomous period ended")
            
    def teleopInit(self) -> None:
        DataLogManager.log("Teleoperated period started")

    def teleopExit(self) -> None:
        DataLogManager.log("Teleoperated period ended")

    def testInit(self):
        DataLogManager.log("Test period started")
        CommandScheduler.getInstance().cancelAll()
        CommandScheduler.getInstance().disable()

    def testExit(self):
        DataLogManager.log("Test period ended")
        CommandScheduler.getInstance().enable()
    
    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass
