from pyb import Pin, I2C
from motor_class import Motor
from encoder_class import Encoder
from IR_Sensor import SensorArray
from bump_sensor import BumpSensorArray
from PID import PIDController
from IMU import IMU
import cotask
import task_share
import math

# Operating Modes
MODE_IDLE = 0
MODE_LINE_FOLLOW = 1
MODE_TURN = 2
MODE_GRID_NAV = 3
MODE_BUMP = 4

# Robot physical parameters
WHEEL_DIAMETER = 70  # mm
WHEEL_BASE = 141   # mm (distance between wheels)
ENCODER_TICKS_PER_REV = 1440  # Encoder ticks per wheel revolution

# Constants for position tracking
MM_PER_TICK = (math.pi * WHEEL_DIAMETER) / ENCODER_TICKS_PER_REV

# Pin Assignments
motor_L_pins = ((2, 3, 'D6'), 'D5', 'D4')
motor_R_pins = ((4, 2, 'B7'), 'D2', 'B0')

encoder_L_pins = (3, 'D12', 'D11')
encoder_R_pins = (1, 'D7', 'D8')

IR_sensor_pins = [('A0', 2139, 3751), 
                  ('A1', 364, 3716), 
                  ('A2', 332, 3744), 
                  ('A4', 299, 3762),
                  ('A5', 357, 3797), 
                  ('C3', 916, 3747)]

bump_L_pins = ['B2']
bump_R_pins = ['B11']

# Initialize the motors
motor_L = Motor(motor_L_pins[0], motor_L_pins[1], motor_L_pins[2])
motor_R = Motor(motor_R_pins[0], motor_R_pins[1], motor_R_pins[2])

encoder_L = Encoder(encoder_L_pins[0], encoder_L_pins[1], encoder_L_pins[2])
encoder_R = Encoder(encoder_R_pins[0], encoder_R_pins[1], encoder_R_pins[2])
    

def motor_task(shares):
    
    effort_L, effort_R, run_state = shares
    
    # State for motor task
    state = 0
    
    while True:
        # State 0: Initialize/Disabled
        if state == 0:
            motor_L.disable()
            motor_R.disable()
            state = 1
        
        # State 1: Disabled - waiting for run command
        elif state == 1:
            # Check if run_state is active
            if run_state.get() == 1:
                motor_L.enable()
                motor_R.enable()
                state = 2 
        
        # State 2: Running - update motor efforts
        elif state == 2:
            # Check if run_state is inactive
            if run_state.get() == 0:
                motor_L.set_effort(0)
                motor_R.set_effort(0)
                state = 3  

            else:
                # Set motor efforts based on shared variables
                left_effort = effort_L.get()
                right_effort = effort_R.get()
                
                # Apply efforts to motors
                motor_L.set_effort(left_effort)
                motor_R.set_effort(right_effort)
        
        # State 3: Braking - stop and disable motors
        elif state == 3:
            # Motors already set to zero effort, now disable
            motor_L.disable()
            motor_R.disable()
            state = 1  
        
        yield state

def line_task(shares):
    
    line_position, mode, effort_L, effort_R, bump_state, run_state, position_threshold = shares
    

    sensor_array = SensorArray(IR_sensor_pins)
    
    pid = PIDController(Kp=8, Ki=.6, Kd=.04, dt=.02)
    
    # Constants for line following
    BASE_SPEED = 25       
    TARGET_POSITION = 3.5  
    
    # State for line following task
    state = 0
    
    # Variables for line following
    line_error = 0
    
    while True:
        # State 0: Initialize
        if state == 0:
            state = 1 
        
        # State 1: Idle - check if we should be line following
        elif state == 1:
            effort_L.put(0)
            effort_R.put(0)
            
            # Check if we should transition to line following
            if mode.get() == MODE_LINE_FOLLOW and run_state.get() == 1:
                state = 2  
        
        # State 2: Following Line - main line following loop
        elif state == 2:
            if run_state.get() == 0:
                effort_L.put(0)
                effort_R.put(0)
                state = 1  

            elif position_threshold.get() > 0 or mode.get() != MODE_LINE_FOLLOW: 
                yield state
            else:
                # Read line sensor data
                readings = sensor_array.read_linearized()
                centroid = sensor_array.get_centroid(readings)

                if centroid is None:
                    state = 2  # Go back to idle (or switch mode if needed)
                else:
                    line_position.put(centroid)  # Only update if cen
                    state = 3  # Calculate PID control
        
        # State 3: Calculate PID Control
        elif state == 3:
            # Get the current centroid position
            centroid = line_position.get()
            
            # Calculate error (difference from target position)
            # Positive error means line is to the left, negative means to the right
            line_error = TARGET_POSITION - centroid
            
            # Move to state 4 to set motor efforts
            state = 4
        
        # State 4: Set Motor Efforts
        elif state == 4:
            # Get the calculated steering value from PID
            steering = pid.calculate(line_error)
            
            # Calculate motor efforts based on steering
            left_effort = BASE_SPEED - steering
            right_effort = BASE_SPEED + steering
            
            # Limit efforts to valid range (0-100)
            left_effort = max(0, min(100, left_effort))
            right_effort = max(0, min(100, right_effort))
            
            # Update effort shares for the motor task
            effort_L.put(left_effort)
            effort_R.put(right_effort)
            
            # Return to line following state
            state = 2
        
        yield state

def bump_task(shares):
    bump_state, mode, position_L, position_R, effort_L, effort_R = shares
    
    # Initialize the bump sensor array
    bump_sensors = BumpSensorArray(bump_L_pins, bump_R_pins)
    
    # State for bump task
    state = 0
    
    while True:
        # State 0: Initialize
        if state == 0:
            bump_state.put(3)  # No collision detected initially
            state = 1
        
        # State 1: Monitor bumpers
        elif state == 1:
            # Read bump sensors
            bump_result = bump_sensors.check_bump()
            
            # Update bump state
            bump_state.put(bump_result)
            
            # If there's a collision, change to state 2
            if bump_result < 3:
                mode.put(MODE_BUMP)
                encoder_L.update()
                pos_L = encoder_L.get_position()
                FINAL_POSITION=pos_L-500
                state = 2
        
        # State 2: Collision handling
        elif state == 2:
            # If collision was resolved, go back to monitoring
            if bump_state.get() == 3:
                state = 1

            else:
                encoder_L.update()
                pos_L = encoder_L.get_position() 

                effort_L.put(-15)
                effort_R.put(-15)

                if pos_L < FINAL_POSITION:
                    # Stop motors
                    effort_L.put(0)
                    effort_R.put(0)
                    state = 3

        elif state == 3:
            current_heading = heading.get()

            # Calculate error between current heading and target heading
            error = initial_heading.get() - current_heading

            if error > 360:
                error -= 360

            # Perform the turn to adjust the heading
            # Variable to check if the turn has already started
            turn_started = False

            # Check if error is greater than the tolerance
            if abs(error) > 4 and not turn_started:
                # Start turning in the correct direction

                    # Turn clockwise (left motor forward, right motor backward)
                effort_L.put(20)
                effort_R.put(-20)

                # Set the flag to indicate the turn has started
                turn_started = True   

            else: 
                effort_L.put(0)
                effort_R.put(0)
                encoder_L.update()
                pos_L = encoder_L.get_position()
                FINAL_POSITION=pos_L+2500
                state = 4
                
        elif state == 4:
            encoder_L.update()
            pos_L = encoder_L.get_position()

            effort_L.put(20)
            effort_R.put(25)

            if pos_L > FINAL_POSITION:
                # Stop motors
                effort_L.put(0)
                effort_R.put(0)

                if initial_heading.get() < 180:
                    target_heading = initial_heading.get() + 180.0
                else:
                    target_heading = initial_heading.get() - 180.0
                    
                    # Make sure target_heading stays in 0-360 range
                    if target_heading >= 360.0:
                        target_heading -= 360.0
                    elif target_heading < 0.0:
                        target_heading += 360.0
                    

                mode.put(MODE_TURN)
                state = 5

        elif state == 5:
            if run_state.get() == 0:
                    # If the run_state is off, stop the motors
                    effort_L.put(0)
                    effort_R.put(0)
            else:
                # Ensure no other task modifies the motor efforts
                if mode.get() != MODE_TURN:
                    effort_L.put(0)
                    effort_R.put(0)
                    state = 1  # Transition to idle if mode is not turn
                
                # Ensure we're still in turn mode
                if mode.get() == MODE_TURN:

                    target_heading_2 =target_heading+90
                    if target_heading_2 >= 360.0:
                        target_heading -= 360.0
                    elif target_heading_2 < 0.0:
                        target_heading_2 += 360.0
                    # Get the current heading
                    current_heading = heading.get()
                    
                    # Calculate error between current heading and target heading
                    error = target_heading_2 - current_heading
                    
                    # Normalize the error to be between -180 and 180 degrees
                    if error > 360:
                        error -= 360
                    
                    # Perform the turn to adjust the heading
                    # Variable to check if the turn has already started
                    turn_started = False

                    # Check if error is greater than the tolerance
                    if abs(error) > 3 and not turn_started:
                        # Start turning in the correct direction                        
                    # Turn counterclockwise (left motor backward, right motor forward)
                        effort_L.put(-20)
                        effort_R.put(20)
                    # Set the flag to indicate the turn has started
                        turn_started = True   
                    
                    else:
            # Stop the motors once the target heading is reached
                        effort_L.put(0)
                        effort_R.put(0)
                        encoder_L.update()
                        pos_L = encoder_L.get_position()
                        FINAL_POSITION=pos_L+1500
                        state = 6

        elif state == 6:
            encoder_L.update()
            pos_L = encoder_L.get_position()

            effort_L.put(20)
            effort_R.put(20)

            if pos_L > FINAL_POSITION:
                # Stop motors
                effort_L.put(0)
                effort_R.put(0)
                state = 7

        elif state == 7:
            if mode.get() == MODE_TURN:
                    # Get the current heading
                    current_heading = heading.get()
                    
                    # Calculate error between current heading and target heading
                    error = target_heading - current_heading
                    
                    # Normalize the error to be between -180 and 180 degrees
                    if error > 360:
                        error -= 360
                    
                    # Perform the turn to adjust the heading
                    # Variable to check if the turn has already started
                    turn_started = False

                    # Check if error is greater than the tolerance
                    if abs(error) > 5 and not turn_started:
                        # Start turning in the correct direction
                            # Turn clockwise (left motor forward, right motor backward)
                        effort_L.put(-20)
                        effort_R.put(20)
                        
                        # Set the flag to indicate the turn has started
                        turn_started = True   
                          
                    else:
             # Stop the motors once the target heading is reached 
                        effort_L.put(0)
                        effort_R.put(0)
                       
                        
                        # After stopping, set the mode to grid navigation or any other mode
                        mode.put(MODE_LINE_FOLLOW)
                        encoder_L.update()
                        pos_L = encoder_L.get_position()
                        FINAL_POSITION=pos_L+2700
            else:
                encoder_L.update()
                pos_L = encoder_L.get_position()
                if pos_L > FINAL_POSITION:
                    position_threshold.put(1)
                    effort_L.put(0)
                    effort_R.put(0)

        yield state

def position_task(shares):
    """
    Task to track robot position, detect checkpoint positions, and navigate to target positions.
    Handles turning around at the checkpoint by using IMU data.
    
    Args:
        shares: Tuple containing shared variables
    """
    position_L, position_R, velocity_L, velocity_R, x_position, y_position, heading, initial_heading, position_threshold, effort_L, effort_R, run_state, mode = shares
    
    # Initialize encoders

    # Constants for position tracking
    DIAMOND_POS = 6500
    GRID_POSITION =25700  # Encoder ticks for first checkpoint
    
    # Motor control constants
    STRAIGHT_NAV_SPEED = 35  # Speed for straight line navigation
    TURN_SPEED = 10  # Speed for turning
    HEADING_TOLERANCE = 1  # Tolerance in degrees for heading alignment
    yaw = PIDController(Kp = 1, Ki = 0, Kd = 0, dt = 0.2)

    # State for position task
    state = 0

    
    target_heading = 0.0  # Heading to maintain during straight line navigation
    import time
    last_debug_time = time.time()

    while True:
        # State 0: Initialize
        if state == 0:
            encoder_L.zero()
            encoder_R.zero()
            x_position.put(0.0)
            y_position.put(0.0)
            position_L.put(0.0)
            position_R.put(0.0)
            velocity_L.put(0.0)
            velocity_R.put(0.0)
            position_threshold.put(0)  # Reset threshold
            state = 1
        
        # State 1: Update position and velocity
        elif state == 1:
            # Only update when run_state is active
            if run_state.get() == 1:
                # Update encoders
                encoder_L.update()
                encoder_R.update()
                  
                # Get position and velocity from encoders
                pos_L = encoder_L.get_position()
                pos_R = encoder_R.get_position()
                vel_L = encoder_L.get_velocity()
                vel_R = encoder_R.get_velocity()
                
                # Update shared variables
                position_L.put(pos_L)
                position_R.put(pos_R)
                velocity_L.put(vel_L)
                velocity_R.put(vel_R)
                
                # Print debug periodically
                current_time = time.time()
                if current_time - last_debug_time > 2.0:
                    last_debug_time = current_time
                
                # Check if reached first checkpoint position
                if pos_L > GRID_POSITION and position_threshold.get() == 0:
                    mode.put(MODE_TURN)  # Switch to dedicated turn mode
                    position_threshold.put(1)  # Set threshold flag
                    
                    # Stop motors
                    effort_L.put(0)
                    effort_R.put(0)
                    
                    # Move to state 2 to prepare for turning
                    state = 2

                if DIAMOND_POS + 1000 > pos_L > DIAMOND_POS and position_threshold.get() == 0:
                    position_threshold.put(1)
                    mode.put(MODE_GRID_NAV)
                    effort_L.put(0)
                    effort_R.put(0)
                    encoder_L.update()
                    pos_L = encoder_L.get_position()
                    FINAL_POSITION=pos_L+1000
                    
                    state = 7
        
        # State 2: Prepare for 180-degree turn
        elif state == 2:
            # Ensure no other task is modifying motor efforts when we are in turn mode
            if mode.get() == MODE_TURN:
                # Make sure we have the initial heading and run_state is active
                if initial_heading.get() != -1 and run_state.get() == 1:
                    # Get the initial heading and the current heading
                    initial_hdg = initial_heading.get()
                    current_hdg = heading.get()
                    
                    # Calculate target heading (180 degrees from initial heading) - OPPOSITE direction
                    if initial_hdg < 180:
                        target_heading = initial_hdg + 180.0
                    else:
                        target_heading = initial_hdg - 180.0
                    
                    # Make sure target_heading stays in 0-360 range
                    if target_heading >= 360.0:
                        target_heading -= 360.0
                    elif target_heading < 0.0:
                        target_heading += 360.0
                    
                    target_heading_2 =target_heading+85
                    if target_heading_2 >= 360.0:
                        target_heading_2 -= 360.0
                    elif target_heading_2 < 0.0:
                        target_heading_2 += 360.0

                    target_heading_3 =target_heading-90
                    if target_heading_3 >= 360.0:
                        target_heading_3 -= 360.0
                    elif target_heading_3 < 0.0:
                        target_heading_3 += 360.0
                    
                    # Add a brief pause to ensure motors stop completely
                  
                    state = 3
        
        # State 3: Execute the 180-degree turn
        elif state == 3:
            if run_state.get() == 0:
                # If the run_state is off, stop the motors
                effort_L.put(0)
                effort_R.put(0)
            else:
                # Ensure no other task modifies the motor efforts
                if mode.get() != MODE_TURN:
                    effort_L.put(0)
                    effort_R.put(0)
                    state = 1  # Transition to idle if mode is not turn
                
                # Ensure we're still in turn mode
                if mode.get() == MODE_TURN:
                    # Get the current heading
                    current_heading = heading.get()
                    
                    # Calculate error between current heading and target heading
                    error = target_heading - current_heading
                    
                    # Normalize the error to be between -180 and 180 degrees
                    if error > 360:
                        error -= 360
                    
                    # Perform the turn to adjust the heading
                    # Variable to check if the turn has already started

                    # Check if error is greater than the tolerance
                    if abs(error) > HEADING_TOLERANCE:
                        # Start turning in the correct direction
                        
                        if error > 0:
                            # Turn counterclockwise (left motor backward, right motor forward)
                            effort_L.put(TURN_SPEED)
                            effort_R.put(-TURN_SPEED)
                        elif error < 0:
                            # Turn clockwise (left motor forward, right motor backward)
                            effort_L.put(-TURN_SPEED)
                            effort_R.put(TURN_SPEED)
                        
                        # Set the flag to indicate the turn has started
                          
                    else:
             # Stop the motors once the target heading is reached
                        effort_L.put(0)
                        effort_R.put(0)

                        # After stopping, set the mode to grid navigation or any other mode
                        mode.put(MODE_GRID_NAV)
                        
                        # Reset the turn_started flag so it doesn't interfere with future turns
                        turn_started = False
                        pos_L = encoder_L.get_position()
                        FINAL_POSITION=pos_L+4500
                        # Transition to the next state (e.g., grid navigation or forward movement)
                        state = 4  # Transition to forward movement after turn completion

        # State 4: Navigate to next checkpoint after turning
        elif state == 4:
            if run_state.get() == 0:
                effort_L.put(0)
                effort_R.put(0)
            else:
                # Ensure we're in grid navigation mode
                if mode.get() != MODE_GRID_NAV:
                    mode.put(MODE_GRID_NAV)
                
                # Update encoders
                encoder_L.update()
                encoder_R.update()
                
                pos_L = encoder_L.get_position()
                pos_R = encoder_R.get_position()
                
                # Update shared variables
                position_L.put(pos_L)
                position_R.put(pos_R)

                current = heading.get()

                error  = target_heading - current
                steering = yaw.calculate(error)
                
                # Set motor efforts for straight line navigation
                effort_L.put(STRAIGHT_NAV_SPEED + steering)
                effort_R.put(STRAIGHT_NAV_SPEED - steering)
                # Check if reached final position
                
                if pos_L > FINAL_POSITION:
                    # Stop motors
                    effort_L.put(0)
                    effort_R.put(0)
                    #position_threshold.put(2)  # Update threshold to indicate target reached
                    mode.put(MODE_TURN)
                    state = 5  # Go to stopped state
        
        # State 5: Stopped at final target
        elif state == 5:
                if run_state.get() == 0:
                    # If the run_state is off, stop the motors
                    effort_L.put(0)
                    effort_R.put(0)
                else:
                    # Ensure no other task modifies the motor efforts
                    if mode.get() != MODE_TURN:
                        effort_L.put(0)
                        effort_R.put(0)
                        state = 1  # Transition to idle if mode is not turn
                    
                    # Ensure we're still in turn mode
                    if mode.get() == MODE_TURN:
                        # Get the current heading
                        current_heading = heading.get()
                        
                        # Calculate error between current heading and target heading
                        error = target_heading_2 - current_heading
                        
                        # Normalize the error to be between -180 and 180 degrees
                        if error > 360:
                            error -= 360
                        
                        # Perform the turn to adjust the heading
                        # Variable to check if the turn has already started
                        turn_started = False

                        # Check if error is greater than the tolerance
                        if abs(error) > 2 and not turn_started:
                            # Start turning in the correct direction
                            
                            if error != 0:
                            # Turn counterclockwise (left motor backward, right motor forward)
                                effort_L.put(15)
                                effort_R.put(-15)

                        # Set the flag to indicate the turn has started
                            turn_started = True   
                        
                        else:
                # Stop the motors once the target heading is reached
                            effort_L.put(0)
                            effort_R.put(0)

                            encoder_L.update()
                            pos_L = encoder_L.get_position()
                            FINAL_POSITION=pos_L+500
                            mode.put(MODE_GRID_NAV)
                        
                                # Transition to the next state (e.g., grid navigation or forward movement)
                            state = 6  # Transition to forward movement after turn completion

        elif state == 6:
            if mode.get() == MODE_GRID_NAV:
                if run_state.get() == 0:
                    effort_L.put(0)
                    effort_R.put(0)
                else:
                    # Ensure we're in grid navigation mode
                    
                    # Update encoders
                    encoder_L.update()
                    encoder_R.update()
                    
                    pos_L = encoder_L.get_position()
                    pos_R = encoder_R.get_position()
                    
                    # Update shared variables
                    position_L.put(pos_L)
                    position_R.put(pos_R)
                    
                    # Set motor efforts for straight line navigation
                    effort_L.put(STRAIGHT_NAV_SPEED)
                    effort_R.put(STRAIGHT_NAV_SPEED)
                    # Check if reached final position
                    
                    if pos_L > FINAL_POSITION:
                        # Stop motors
                        effort_L.put(0)
                        effort_R.put(0)
                        position_threshold.put(0)
                        mode.put(MODE_LINE_FOLLOW)

        elif state == 7:

            target_heading_3 =target_heading+90
            if target_heading_3 >= 360.0:
                target_heading_3 -= 360.0
            elif target_heading_3 < 0.0:
                target_heading_3 += 360.0

            if run_state.get() == 0:
                effort_L.put(0)
                effort_R.put(0)
            else:

                current_heading = heading.get()
                    
                    # Calculate error between current heading and target heading
                error = target_heading_3 - current_heading
                
                # Normalize the error to be between -180 and 180 degrees
                if error > 360:
                    error -= 360
                
                # Perform the turn to adjust the heading
                # Variable to check if the turn has already started
                turn_started = False

                # Check if error is greater than the tolerance
                if abs(error) > 2 and not turn_started:
                    # Start turning in the correct direction
                    
                    if error != 0:
                    # Turn counterclockwise (left motor backward, right motor forward)
                        effort_L.put(15)
                        effort_R.put(-15)

                    turn_started = True
                
                else:
        # Stop the motors once the target heading is reached
                    effort_L.put(0)
                    effort_R.put(0)
                    turn_started = False

                    state = 8

        elif state == 8:
                
            # Update encoders
            encoder_L.update()
            encoder_R.update()
            
            pos_L = encoder_L.get_position()
            pos_R = encoder_R.get_position()
            
            # Update shared variables
            position_L.put(pos_L)
            position_R.put(pos_R)
            
            # Set motor efforts for straight line navigation
            effort_L.put(20)
            effort_R.put(20)
            # Check if reached final position
            
            if pos_L > FINAL_POSITION:
                # Stop motors
                effort_L.put(0)
                effort_R.put(0)

                position_threshold.put(0)
                mode.put(MODE_LINE_FOLLOW)
                state = 1
    
        yield state

def imu_task(shares):
    """
    Task to read orientation data from the BNO055 IMU.
    
    Args:
        shares: Tuple containing (heading, initial_heading, is_calibrated)
    """
    
    # Unpack shares
    heading, initial_heading = shares
    
    # State for IMU task
    state = 0
    
    # Variables for IMU task
    imu = None
    heading_value = 0.0
    
    while True:
        # State 0: Initialize IMU
        if state == 0:
            try:
                # Initialize I2C with correct pins for Nucleo
                i2c = I2C(1, I2C.CONTROLLER)
                i2c.init(I2C.CONTROLLER, baudrate=100000)
                
                # Initialize BNO055 with I2C
                imu = IMU(i2c)
                
                state = 1
                
            except Exception as e:
                # Retry after a delay
                pass
        
        # State 1: Read heading and update shares
        elif state == 1:
            try:
                # Read heading from IMU (0-360 degrees)
                heading_value = imu.get_heading()
                
                # Update shared heading value
                heading.put(heading_value)
                
                # Store initial heading if not set yet
                if initial_heading.get() == -1:
                    initial_heading.put(heading_value)
            
            except Exception as e:
                state = 0  
        
        yield state

if __name__ == "__main__":

    # Shared variables
    position_L = task_share.Share('f', thread_protect=True, name="Position_L")
    position_R = task_share.Share('f', thread_protect=True, name="Position_R")
    velocity_L = task_share.Share('f', thread_protect=True, name="Velocity_L")
    velocity_R = task_share.Share('f', thread_protect=True, name="Velocity_R")
    effort_L = task_share.Share('f', thread_protect=True, name="Effort_L")
    effort_R = task_share.Share('f', thread_protect=True, name="Effort_R")
    line_position = task_share.Share('f', thread_protect=True, name="Line_Position")
    heading = task_share.Share('f', thread_protect=True, name="Heading")
    initial_heading = task_share.Share('f', thread_protect=True, name="Initial_Heading")  # Fixed name
    bump_state = task_share.Share('B', thread_protect=True, name="Bump_State")
    mode = task_share.Share('B', thread_protect=True, name="Mode")
    run_state = task_share.Share('B', thread_protect=True, name="Run_State")
    x_position = task_share.Share('f', thread_protect=True, name="X_Position")
    y_position = task_share.Share('f', thread_protect=True, name="Y_Position")
    position_threshold = task_share.Share('i', thread_protect=True, name="Position_Threshold_1")

    # Initialize all shared variables
    heading.put(0.0)
    initial_heading.put(-1.0)  # -1 indicates not set yet
    position_threshold.put(0)

    mode.put(MODE_LINE_FOLLOW)
    run_state.put(0)

    # Create the tasks with proper priorities
    motor_task = cotask.Task(motor_task, name="Motor Task", priority=2, 
                             period=10, profile=True, trace=False,
                             shares=(effort_L, effort_R, run_state))
    
    line_task = cotask.Task(line_task, name="Line Task", priority=3, 
                           period=10, profile=True, trace=False,
                           shares=(line_position, mode, effort_L, effort_R, 
                                   bump_state, run_state, position_threshold))
    
    position_task = cotask.Task(position_task, name="Position Task", priority=4,  # Higher priority
                               period=15, profile=True, trace=False,
                                shares=(position_L, position_R, velocity_L, velocity_R, 
                                       x_position, y_position, heading, initial_heading, 
                                       position_threshold, effort_L, effort_R, run_state, mode))  # Added mode
    
    imu_task = cotask.Task(imu_task, name="IMU Task", priority=1, 
                          period=50, profile=True, trace=False,
                          shares=(heading, initial_heading))
    
    bump_task = cotask.Task(bump_task, name="Bump Task", priority=5, 
                            period=50, profile=True, trace=False,
                            shares=(bump_state, mode, position_L, position_R, 
                                    effort_L, effort_R))
    
    
    # Add only the tasks we need
    cotask.task_list.append(motor_task)
    cotask.task_list.append(line_task)
    cotask.task_list.append(position_task)
    cotask.task_list.append(imu_task)
    cotask.task_list.append(bump_task)

    #memory garbage collector
    import gc
    gc.collect()
   
    # User button for starting/stopping
    user_button = Pin('C13', Pin.IN, Pin.PULL_DOWN)
    
    # Set mode to line following initially
    mode.put(MODE_LINE_FOLLOW)

    heading.put(0.0)
    initial_heading.put(-1.0)
    
    
    last_button_state = 0
    
    while True:

        try:
        # Check if button is pressed
            button_state = user_button.value()
            if button_state == 1 and last_button_state == 0:
                if run_state.get() == 0:
                    button_state = ()
                    run_state.put(1)
                else:
                    run_state.put(0)
                    
            last_button_state = button_state
            
            # Run the scheduler
            cotask.task_list.pri_sched()
        
        except KeyboardInterrupt:
            break

