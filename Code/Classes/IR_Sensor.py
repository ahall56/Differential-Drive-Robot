from pyb import ADC

class SingleSensor:
    """Class to interface with a single IR sensor.
    Arguments:
            -- A tuple, Pin.
                - item 0: Analog pin associated with sensor. [str]
                - Pre-determined white calibration value. [int]
                - Pre-determined black calibration value. [int]
    """
    
    def __init__(self, pin):
        """Initializes single IR sensor."""
        
        self.sensor = ADC(pin[0])
        self.white = pin[1]
        self.black = pin[2]
        self.reading = 0
        self.linear = 0
    
    def get_reading(self):
        """Returns a reading from a single sensor."""
    
        self.reading = self.sensor.read()
        return self.reading
    
    def interpolate(self, reading):
        """Takes in sensor reading and linearizes the value between 0 and 1 
        based on the black and white calibration values.
        Arguments:
                -- Reading from single IR sensor.
        """
       
        self.linear = (reading - self.white) / (self.black - self.white)

        if self.linear < 0:
            self.linear = 0
        elif self.linear > 1:
            self.linear = 1
        
        return self.linear
    
class SensorArray:
    """Class to interface with a series of IR sensors.
    Arguments:
            -- An array of anolog pins associated with the sensor array. (array
            should be fromatted from left sensor to right sensor.)
    """
    
    def __init__(self, pins):
        """Initializes array of IR sensors."""

        self.array = []
        self.readings = []
        
        for pin in pins:
            sensor = SingleSensor(pin)
            self.array.append(sensor)

    def read_linearized(self):
        """Gets a reading and linearizes it for each sensor in the array."""
 
        self.readings.clear()
        for sensor in self.array:
            reading = sensor.get_reading()
            self.readings.append(sensor.interpolate(reading))

        return self.readings
    
    def get_centroid(self, readings):
        """Returns the centroid from a set of linearized readings from the 
        sensor array.
        Arguments:
                -- Array of readings from each sensor.
        """

        numerator = 0
        denominator = 0
    
        for i, value in enumerate(readings):
            numerator += value * (i+1)
            denominator += value
            
        if denominator == 0:
            return None  # No line detected
        return numerator / denominator
    
    def get_sum(self, readings):

        sum_val = sum(readings)

        return sum_val
    
    
