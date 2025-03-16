from pyb import Timer, Pin
from time import ticks_us, ticks_diff

class Encoder:
    """ A quadrature encoder decoding interface encapsulated in a class. 
    Arguments:
            -- Timer to be set to encoder mode
            -- Channel A pin for the timer
            -- Channel B pin for the timer
            -- Auto reload value
    """

    def __init__(self, tim, chA_pin, chB_pin, AR = 0xFFFF):
        """ Initializes an encoder object """

        # create encoder objects
        self.ENC_tim = Timer(tim, period = 0xFFFF, prescaler = 0)
        self.ENC_tim.channel(1, pin=Pin(chA_pin), mode=Timer.ENC_AB)
        self.ENC_tim.channel(2, pin=Pin(chB_pin), mode=Timer.ENC_AB)

        # initialize encoder data
        self.position   = 0
        self.prev_count = self.ENC_tim.counter()
        self.delta      = 0
        self.dt         = 0
        self.prev_time = ticks_us()
        self.AR = AR

    def update(self):
        """ Runs one update step on the encoder's timer counter to keep
            track of the change in count and check for counter reload """

        # get current count and time
        current_count = self.ENC_tim.counter()
        current_time = ticks_us()

        self.delta = current_count - self.prev_count    # encoder counts

        # correct for auto reload underflow/overflow
        half_AR = (self.AR+1) // 2
        if self.delta > half_AR:
            self.delta -= self.AR + 1

        elif self.delta < -half_AR:
            self.delta += self.AR + 1

        # update encoder data
        self.position  += self.delta
        self.dt = ticks_diff(current_time, self.prev_time) / 1_000_000 # in sec
        self.prev_count = current_count
        self.prev_time = current_time

    def get_position(self):
        """ Returns the most recently updated value of position as determined 
        within the update() method """
        
        return self.position
    
    def get_velocity(self):
        """ Returns a measure of velocity using the most recently updated 
            value of delta as determined within the update() method """
        
        return self.delta/self.dt if self.dt > 0 else 0
    
    def zero(self):
        """ Sets the present encoder position to zero and causes future updates
            to measure with respect to the new zero position """
        
        self.position = 0
        self.prev_count = self.ENC_tim.counter()
