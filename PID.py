
class PIDController:
    """Class for PID control of motos or position.
    Arguments:
            -- Proportional gain [int]
            -- Integral gain [int]
            -- Derivative gain [int]
    """
    
    def __init__(self, Kp, Ki, Kd):
        """Initializes PID object with desired gains."""

        self.Kp = Kp  
        self.Ki = Ki  
        self.Kd = Kd  
        
        self.previous_error = 0
        self.integral = 0

    def calculate(self, error, run_state):
        """Calculated PID control to send to control plant based on error.
        Arguments:
                -- Previously calculated error for position or velocity. [int]
                -- Flag to check if system is running or not. [byte]
        """

        # If running, calculate PID action
        if run_state == 1:
            P = self.Kp * error

            self.integral += error
            I = self.Ki * self.integral

            D = self.Kd * (error - self.previous_error)
    
            self.previous_error = error

        # If not running, set PID action to 0
        else:
            P=0
            I=0
            D=0
        
        return P + I + D
    
