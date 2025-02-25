from pyb import Timer, Pin

class Motor:
    """ 
    Motor driver class. Works with motor drivers useing seperate PWM,
    direction, and enable pin inputs. 
    Arguments:
            -- tuple, PWM, which contains the specific timer, channel, and
               associated pin. 
            -- The direction pin for the motor.
            -- The enale pin for the motor.
    """
    def __init__(self, PWM, DIR, ENA):
        """Initializes a Motor object"""

        self.ENA_pin = Pin(ENA, mode=Pin.OUT_PP, value=0)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)
        self.PWM_tim = Timer(PWM[0], freq=1000)
        self.PWM_pin = self.PWM_tim.channel(PWM[1], pin=Pin(PWM[2]), 
                                            mode=Timer.PWM, 
                                            pulse_width_percent=0)

    def set_effort(self, effort):
        """ Sets the percent effort requested from the motor based on an 
            input value between -100 and 100 """
        
        if effort >= 0:
            self.DIR_pin.value(0)
            self.PWM_pin.pulse_width_percent(effort)

        else:
            self.DIR_pin.value(1)
            effort = -effort
            self.PWM_pin.pulse_width_percent(effort)

    def enable(self):
        """ Enables the motor driver by taking it out of sleep moe into 
        brake mode """
        
        self.ENA_pin.high()
        self.PWM_pin.pulse_width_percent(0)

    def disable(self):
        """ Disables the motor driver by taking it into sleep mode """
        
        self.ENA_pin.low()