from commands2 import Subsystem
from constants import Constants
import ntcore
import math
from subsystems.drivetrain import Drivetrain
from wpimath.kinematics import ChassisSpeeds
from wpilib import DriverStation

class AutoAlign(Subsystem):

    def __init__(self, drivetrain: Drivetrain):

        self.drivetrain = drivetrain
        self.ntInstance = ntcore.NetworkTableInstance.getDefault()
        # self.addRequirements(self.drivetrain)

    def calculateDegrees(self):

        # read from LimeLight
        # know which alliance
        # confirm correct tag
        # calculate the angle to speaker
        # ! calculate the hypot for distance
        # convert degrees to rotations per constant of gear ratio
        # return number of rotations
        # ! calculate the rev speed

        """
        To do

        Set id priority on LimeLight (In progress)

        """
        table = self.ntInstance.getTable("limelight")
        # NetworkTables.getTable("limelight").putNumber('priorityid',4) I have a topic on this pending in Chief Delphi so I'll know how to write this.
        targetOffsetAngle = table.getNumber("ty",0.0)
        tagId = table.getNumber("tid", 0)
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if tagId != Constants.LimeLight.REDSPEAKERID: #need ids for blue or red and also check that
                return 0
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if tagId != Constants.LimeLight.BLUESPEAKERID: #need ids for blue or red and also check that
                return 0
        
        
       
        angleToTargetRadians = self.getAngleToTargetInRadians(targetOffsetAngle)
        distanceToGoal = self.getDistanceToTargetInches(angleToTargetRadians)
        degrees = self.getDegreesToSpeaker(distanceToGoal)

        return (degrees * Constants.Swivel.GEAR_RATIO) / 360

    def getAngleToTargetInRadians(self, targetOffsetAngle):
        angleToGoalDegrees = Constants.LimeLight.k_mount_angle  + targetOffsetAngle
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0)
        return angleToGoalRadians
    
    # Would only be used if we want to adjust velocity of launcher or determine if we are too close or far
    def getDistanceToTargetInches(self, angleToTargetRadians):
        return (Constants.LimeLight.k_tag_height - Constants.LimeLight.k_mount_height)/math.tan(angleToTargetRadians)
    
    def getDegreesToSpeaker(self, distance):
        if distance <= 0.0:
            return Constants.Swivel.MAX_ANGLE
        degrees = math.degrees(math.atan(Constants.LimeLight.k_target_height/distance))
        if degrees > Constants.Swivel.MAX_ANGLE:
            return Constants.Swivel.MAX_ANGLE
        return degrees


    def isFinished(self):

        return False
