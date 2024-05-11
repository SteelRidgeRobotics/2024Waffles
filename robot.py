from commands2 import TimedCommandRobot
from container import RobotContainer

class Waffles(TimedCommandRobot):

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)
        
    def robotInit(self) -> None:
        self.container = RobotContainer()
    
    # Most of these are all here to suppress warnings
    def robotPeriodic(self) -> None:
        pass
    
    def _simulationPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        
        # Reset gyro
        self.container.drivetrain.reset_yaw()
        
        # Get the selected auto
        selected_auto = self.container.get_selected_auto()
        
        # If it's None, do nothing, otherwise schedule the auto.
        if selected_auto is None:
            print("No Auto Selected, doing nothing :(")
        else:
            selected_auto.schedule()
            
            # Show trajectory on field widget
            self.container.drivetrain.load_auto_trajectory(selected_auto)
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        
        # Clear the auto trajectory when we leave auto.
        self.container.drivetrain.clear_auto_trajectory()
            
    def teleopInit(self) -> None:
        pass
    
    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass
