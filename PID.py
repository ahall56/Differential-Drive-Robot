
class PIDController:
    """PID controller to control motor steering based on a centroid input"""
    
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp  
        self.Ki = Ki  
        self.Kd = Kd  
        self.dt = dt
        
        self.previous_error = 0
        self.integral = 0

    def calculate(self, error):
        """output based on the error"""

        P = self.Kp * error

        self.integral += error * self.dt
        I = self.Ki * self.integral

        D = self.Kd * (error - self.previous_error) / self.dt

        self.previous_error = error
        
        #output
        return P + I + D
    
