"""
Common conversions used throughout the robot.
"""

from constants import Constants
import math

def rot_to_meters(rotations: float) -> float:
    """Converts rotations into meters."""
    
    wheel_circumference = math.pi * Constants.Drivetrain.k_wheel_size
    return rotations * wheel_circumference

def meters_to_rots(meters: float) -> float:
    """Converts meters into rotations."""
    
    wheel_circumference = math.pi * Constants.Drivetrain.k_wheel_size
    return meters / wheel_circumference

def rots_to_degs(rotation: float) -> float:
    """Converts rotations to degrees."""
    
    return rotation * 360

def degs_to_rots(degrees: float) -> float:
    """Converts degrees to rotations."""
    
    return degrees / 360
