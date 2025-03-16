from pyb import Pin

class BumpSensor:
    """ Class to interface with a single bump sensor.
    Arguments:
            -- Pin associated with bump sensor. [str]
    """

    def __init__(self, pin):
        """Initializes single bump sensor."""

        self.sensor = Pin(pin, mode = Pin.PULL_UP)

    def check_bump(self):
        """Returns True if sensor is pressed, otherwise returns False"""

        reading = self.sensor.value()

        return reading 
    
class BumpSensorArray:
    """Class to interface with array of bump sensors.
    Arguments:
            -- Array of pins associated with left bump sensors. (Format array 
            left to right) [str]
            -- Array of pins associated with right bump sensors. (Format array 
            left to right) [str]
    """

    def __init__(self, left_pins, right_pins):
        """Initializes array of left and right sensors."""

        self.left = []
        for pin in left_pins:
            sensor_L = BumpSensor(pin)
            self.left.append(sensor_L)

        self.right = []
        for pin in right_pins:
            sensor_R = BumpSensor(pin)
            self.right.append(sensor_R)

    def check_bump(self):
        """Checks if all sensors are pressed, and check if bump is on left side,
        right side, or both.
        Returns:
            0: No collision
            1: Left side
            2: Right side
            3: Both sides
        """

        left_bump = any(sensor.check_bump() for sensor in self.left)
        right_bump = any(sensor.check_bump() for sensor in self.right)

        if left_bump and right_bump:
            return 3
        
        elif right_bump:
            return 2
        
        elif left_bump:
            return 1
        
        else:
            return 0
        
    def is_collision(self):
        """Returns True if any sensor is pressed, False otherwise."""
        return self.check_collision() > 0
    
    def is_left_collision(self):
        """Returns True if any left sensor is pressed, False otherwise."""
        return any(sensor.is_pressed() for sensor in self.left_sensors)
    
    def is_right_collision(self):
        """Returns True if any right sensor is pressed, False otherwise."""
        return any(sensor.is_pressed() for sensor in self.right_sensors)
