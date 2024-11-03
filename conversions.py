from constants import Constants
import math

def rot_to_meters(rotations: float) -> float:
    """Converts rotations into meters."""
    
    wheel_circumference = math.pi * Constants.Drivetrain.k_wheel_diameter
    return rotations * wheel_circumference

def meters_to_rots(meters: float) -> float:
    """Converts meters into rotations."""
    
    wheel_circumference = math.pi * Constants.Drivetrain.k_wheel_diameter
    return meters / wheel_circumference

def rots_to_degs(rotation: float) -> float:
    """Converts rotations to degrees."""
    
    return rotation * 360

def degs_to_rots(degrees: float) -> float:
    """Converts degrees to rotations."""
    
    return degrees / 360

def degs_to_rads(degrees: float) -> float:
    """Converts degrees to radians."""

    return degrees * (math.pi/180)

def rads_to_degs(radians: float) -> float:
    """Converts radians to degrees."""
    return radians * (180/math.pi)

def rads_to_rots(radians: float) -> float:
    return radians / (2 * math.pi)

def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamps value to be between the minimum and maximum value."""
    return max(min_value, min(value, max_value))
