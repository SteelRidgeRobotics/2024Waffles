import commands2
import constants
import wpilib
import ntcore
from subsystems.drivetrain import Drivetrain
from wpimath.kinematics import ChassisSpeeds

class AutoAlign(commands2.Command):

    def __init__(self, drivetrain: Drivetrain):

        self.drivetrain = drivetrain
        self.ntInstance = ntcore.NetworkTableInstance.getDefault()
        self.addRequirements(self.drivetrain)

    def execute(self):

        xyzArray = self.ntInstance.getTable("limelight").getNumberArray("camerapose_targetspace", [4, 4, 4, 4, 4, 4])
        
        wpilib.SmartDashboard.putNumberArray("XYZ", xyzArray)
        wpilib.SmartDashboard.putNumber("X", xyzArray[0])
        wpilib.SmartDashboard.putNumber("Y", xyzArray[1])
        wpilib.SmartDashboard.putNumber("Z", xyzArray[2])
        wpilib.SmartDashboard.putNumber("Roll", xyzArray[3])
        wpilib.SmartDashboard.putNumber("Pitch", xyzArray[4])
        wpilib.SmartDashboard.putNumber("Yaw", xyzArray[5])

        tv = self.ntInstance.getTable("limelight").getNumber("tv", 0.0)
        wpilib.SmartDashboard.putNumber("TV", tv)
        if tv == 1:
            wpilib.SmartDashboard.putString("Target?", "Yes")
        else:
            wpilib.SmartDashboard.putString("Target?", "Nuh uh")
        
                
        #Trying to see if I can code the alignment
        tx = self.ntInstance.getTable("limelight").getNumber("tx", 0.0)
        if tx >= 7:
            wpilib.SmartDashboard.putString("Facing?", "Too far left")
        elif tx <= -7:
            wpilib.SmartDashboard.putString("Facing?", "Too far right")
        else:
            wpilib.SmartDashboard.putString("Facing?", "Straight ahead")

        #self.drivetrain.drive_robot_centric(ChassisSpeeds(0, 0, tx * -constants.Constants.Limelight.k_auto_align_kp))

    def isFinished(self):

        return False