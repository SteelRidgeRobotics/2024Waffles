from commands2 import TimedCommandRobot
from container import RobotContainer
from elastic import *
from wpilib import CameraServer, DataLogManager, DriverStation, RobotBase
from wpimath.geometry import Pose2d, Rotation2d

class Waffles(TimedCommandRobot):

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)
        
    def robotInit(self) -> None:
        self.container = RobotContainer()

        self.addPeriodic(self.container.drivetrain.update_odometry, 0.001)
        self.addPeriodic(self.container.drivetrain.update_vision_estimates, 0.1, 0.01)

        DriverStation.silenceJoystickConnectionWarning(not DriverStation.isFMSAttached())

        if RobotBase.isReal():
            DataLogManager.start("/home/lvuser/logs")
        else:
            DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        if RobotBase.isReal():
            CameraServer.launch()

        DataLogManager.log("robotInit finished")

    # Most of these are all here to suppress warnings
    def robotPeriodic(self) -> None:
        pass
    
    def _simulationPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        DataLogManager.log("Autonomous period started")
        
        # Reset gyro
        self.container.drivetrain.reset_yaw()

        # If we're on the red alliance, rotate the odometry 180 degrees
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.container.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d.fromDegrees(180)))
        else:
            self.container.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        
        # Get the selected auto
        selected_auto = self.container.get_selected_auto()
        
        # If it's None, do nothing, otherwise schedule the auto.
        if selected_auto is None:
            DataLogManager.log("No Auto Selected, doing nothing :(")
            Elastic.send_alert(ElasticNotification(NotificationLevel.INFO, "Autonomous Start", "Scheduled selected auto: Nothing :("))
        else:
            selected_auto.schedule()
            DataLogManager.log(f"Scheduled selected auto: {selected_auto.getName()}")
            Elastic.send_alert(ElasticNotification(NotificationLevel.INFO, "Autonomous Start", f"Scheduled selected auto: {selected_auto.getName()}"))
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        DataLogManager.log("Autonomous period ended")

        if DriverStation.isFMSAttached():
            Elastic.send_alert(ElasticNotification(NotificationLevel.INFO, "Autonomous End", "Good luck!"))
            
    def teleopInit(self) -> None:
        DataLogManager.log("Teleoperated period started")

    def teleopExit(self) -> None:
        DataLogManager.log("Teleoperated period ended")
    
    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass
