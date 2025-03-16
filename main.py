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

#Modes
MODE_IDLE = 0
MODE_LINE_FOLLOW = 1
MODE_TURN = 2
MODE_GRID_NAV = 3
MODE_BUMP = 4

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

# Motor Init
motor_L = Motor(motor_L_pins[0], motor_L_pins[1], motor_L_pins[2])
motor_R = Motor(motor_R_pins[0], motor_R_pins[1], motor_R_pins[2])

encoder_L = Encoder(encoder_L_pins[0], encoder_L_pins[1], encoder_L_pins[2])
encoder_R = Encoder(encoder_R_pins[0], encoder_R_pins[1], encoder_R_pins[2])
    

def motor_task(shares):
    """
    Task designed to control motor comnditions including disabled, running, and braking. 
    It achieves this by using the motor class andassigning efforts.
    
    """
    
    effort_L, effort_R, run_state = shares
    
    # Reset
    state = 0
    
    while True:
        
        # State 0: Init
        if state == 0:
            if run_state.get == 1:
                motor_L.enable()
                motor_R.enable()
                state = 1
        
        
        # State 2: Running
        elif state == 1:
            #is run_state flag downe?
            if run_state.get() == 0:
                motor_L.set_effort(0)
                motor_R.set_effort(0)

            else:
                #extract motor efforts
                left_effort = effort_L.get()
                right_effort = effort_R.get()
                
                #update motor efforts
                motor_L.set_effort(left_effort)
                motor_R.set_effort(right_effort)
        
        
        yield state

def line_task(shares):
    """
    Task designed to adjust motor efforts based on a PID controller that uses IR sensors 
    to detect where the line is realtive the the line sensor strip.
    
    """
    line_position, mode, effort_L, effort_R, run_state, position_threshold = shares

    #define sensors
    sensor_array = SensorArray(IR_sensor_pins)
    
    #define line fowllowing parameters
    pid = PIDController(Kp=8, Ki=.6, Kd=.04, dt=.02) #PID Gains for line following
    BASE_SPEED = 25       #default Speed   
    TARGET_POSITION = 3.5 #centroid (sensors count/2)  
    
    # Reset
    state = 0
  
    line_error = 0
    
    while True:

        # State 0: Init
        if state == 0:
            state = 1 
        
        # State 1: Idle
        elif state == 1:
            effort_L.put(0)
            effort_R.put(0)
            
            # Is run state on & in line follow mode
            if mode.get() == MODE_LINE_FOLLOW and run_state.get() == 1:
                state = 2  
        
        # State 2: Line Detect
        elif state == 2:
            #return to idle if run isnt on
            if run_state.get() == 0:
                effort_L.put(0)
                effort_R.put(0)
                state = 1  
            #stay in state if past position threshold or in line follow mode
            elif position_threshold.get() > 0 or mode.get() != MODE_LINE_FOLLOW: 
                yield state
            else:
                #read line sensors
                readings = sensor_array.read_linearized()
                centroid = sensor_array.get_centroid(readings)

                if centroid is None:
                    state = 2  #return to idle if no line detected
                else:
                    line_position.put(centroid)  #line is centroid
                    state = 3  
        
        # State 3: PID Control
        elif state == 3:
            # centroid is line
            centroid = line_position.get()

            #error, where positive means to the left and negative means to the right
            line_error = TARGET_POSITION - centroid
            state = 4
        
        # State 4: Set Efforts
        elif state == 4:
            #from PID class get error
            steering = pid.calculate(line_error)
            
            #adjust motor efforts based on error
            left_effort = BASE_SPEED - steering
            right_effort = BASE_SPEED + steering
            
            #limit effort range (0-100)
            left_effort = max(0, min(100, left_effort))
            right_effort = max(0, min(100, right_effort))
            
            #update effort
            effort_L.put(left_effort)
            effort_R.put(right_effort)
            
            state = 2
        
        yield state

def bump_task(shares):
    """
    Task designed to trigger a sequence of turns and straight line segments based on specific directional 
    headings and ecoder counts. This sequence is triggered by limit switches to detect the wall bump.
    
    """
    bump_state, mode, effort_L, effort_R = shares
    
    #define limit switches as bumpers
    bump_sensors = BumpSensorArray(bump_L_pins, bump_R_pins)
    
    # Reset
    state = 0
    
    while True:

        # State 0: Init
        if state == 0:
            bump_state.put(3)
            state = 1
        
        # State 1: Wait for Bump
        elif state == 1:
            #read bump sensors
            bump_result = bump_sensors.check_bump()
            
            #update bump state
            bump_state.put(bump_result)
            
            #if bump detected change set mode to bump and go to next state
            if bump_result < 3:
                mode.put(MODE_BUMP)
                encoder_L.update()
                pos_L = encoder_L.get_position()
                FINAL_POSITION=pos_L-500 #defines ecoder position relative to cutrrent location 
                state = 2
        
        # State 2: Collision handling
        elif state == 2:
            #false bump detected
            if bump_state.get() == 3:
                state = 1

            #back up distance based on last encoder count
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

        # State 3: Bump Turn 1
        elif state == 3:

            #get error based on initial heading and current heading
            current_heading = heading.get()
            error = initial_heading.get() - current_heading

            if error > 360:
                error -= 360

            #turning already?
            turn_started = False

            #is error between tolerance
            if abs(error) > 4 and not turn_started:
                
                #set efforts for turn
                effort_L.put(20)
                effort_R.put(-20)

                #started turning
                turn_started = True   
            
            #error within tolerance and romi is facing the correct direction
            else: 
                effort_L.put(0)
                effort_R.put(0)
                encoder_L.update()
                pos_L = encoder_L.get_position()
                FINAL_POSITION=pos_L+2500 #defines ecoder position relative to cutrrent location 
                state = 4

        # State 4:Bump Drive 1        
        elif state == 4:
            #update position
            encoder_L.update()
            pos_L = encoder_L.get_position()

            #Define efforts
            effort_L.put(20)
            effort_R.put(25)

            #reached defined distance
            if pos_L > FINAL_POSITION:
                #stop
                effort_L.put(0)
                effort_R.put(0)

                #Get next turns heading
                if initial_heading.get() < 180:
                    target_heading = initial_heading.get() + 180.0
                else:
                    target_heading = initial_heading.get() - 180.0
                    
                    # Make sure target_heading stays in 0-360 range
                    if target_heading >= 360.0:
                        target_heading -= 360.0
                    elif target_heading < 0.0:
                        target_heading += 360.0
                    
                #set mode to turn
                mode.put(MODE_TURN)
                state = 5

        # State 5: Bump Turn 2
        elif state == 5:
            #stop motors if not in run
            if run_state.get() == 0:
                    effort_L.put(0)
                    effort_R.put(0)
            else:
                #if we are in turn mode stop and return to wait for bump state
                if mode.get() != MODE_TURN:
                    effort_L.put(0)
                    effort_R.put(0)
                    state = 1  
                
                #if we are in turn mode define next heading
                if mode.get() == MODE_TURN:

                    target_heading_2 =target_heading+90
                    if target_heading_2 >= 360.0:
                        target_heading -= 360.0
                    elif target_heading_2 < 0.0:
                        target_heading_2 += 360.0
               
                    current_heading = heading.get()
                    
                    #get error based on initial heading and current heading
                    error = target_heading_2 - current_heading
                    
                    if error > 360:
                        error -= 360
                    
                    turn_started = False
                    
                    #error within tolerance and romi is facing the correct direction
                    if abs(error) > 3 and not turn_started:
                   
                        effort_L.put(-20)
                        effort_R.put(20)
                        turn_started = True   
                    
                    else:
                        effort_L.put(0)
                        effort_R.put(0)
                        encoder_L.update()
                        pos_L = encoder_L.get_position()
                        FINAL_POSITION=pos_L+1500   #defines ecoder position relative to cutrrent location
                        state = 6
 
        # State 6:Bump Drive 2
        elif state == 6:
            #track position and set motor efforts
            encoder_L.update()
            pos_L = encoder_L.get_position()

            effort_L.put(20)
            effort_R.put(20)

            #stop once set distance is reached
            if pos_L > FINAL_POSITION:
                effort_L.put(0)
                effort_R.put(0)
                state = 7

        # State 7:Bump Turn 3 
        elif state == 7:
            if mode.get() == MODE_TURN:
                    
                    #get error based on targetr heading and current heading
                    current_heading = heading.get()
                    error = target_heading - current_heading
  
                    if error > 360:
                        error -= 360

                    turn_started = False

                    #turn until romi is facing the correct direction
                    if abs(error) > 5 and not turn_started:
                        effort_L.put(-20)
                        effort_R.put(20)
                        turn_started = True   
                          
                    else:
                        effort_L.put(0)
                        effort_R.put(0)
                       
                        
                        #set mode to line follow so Romi may use the line following to return to start/finish
                        mode.put(MODE_LINE_FOLLOW)
                        encoder_L.update()
                        pos_L = encoder_L.get_position()
                        FINAL_POSITION=pos_L+2700   #defines ecoder position relative to cutrrent location
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
    heading, initial_heading, position_threshold, effort_L, effort_R, run_state, mode = shares

    #Position Parameters
    DIAMOND_POS = 6500     #encoder ticks for diamond checkpoint
    GRID_POSITION =25700  #encoder ticks for grid checkpoint
    
    STRAIGHT_NAV_SPEED = 35  #default speed
    TURN_SPEED = 10  #turn speed
    HEADING_TOLERANCE = 1  #tolerance in degrees for heading
    yaw = PIDController(Kp = 1, Ki = 0, Kd = 0, dt = 0.2)   #grid navigationP PID controller

    # Reset
    state = 0
    target_heading = 0.0 

    while True:

        # State 0: Init
        if state == 0:
            encoder_L.zero()
            encoder_R.zero()
            position_threshold.put(0)
            state = 1
        
        # State 1: Which Checkpoint?
        elif state == 1:
            
            #update if run is on
            if run_state.get() == 1:

                encoder_L.update()
                encoder_R.update()
               
                #store positions form encoder
                pos_L = encoder_L.get_position()
                pos_R = encoder_R.get_position()

                #are we at the grid, if so stop
                if pos_L > GRID_POSITION and position_threshold.get() == 0:
                    mode.put(MODE_TURN)  #set to turn mode
                    position_threshold.put(1)
                    effort_L.put(0)
                    effort_R.put(0)

                    state = 2
                
                #are we at the diamond, if so stop
                if DIAMOND_POS + 1000 > pos_L > DIAMOND_POS and position_threshold.get() == 0:
                    position_threshold.put(1)
                    mode.put(MODE_GRID_NAV)
                    effort_L.put(0)
                    effort_R.put(0)
                    encoder_L.update()
                    pos_L = encoder_L.get_position()
                    FINAL_POSITION=pos_L+1000   #defines ecoder position relative to cutrrent location
                    
                    state = 7 
        
        # State 2: Grid Prep
        elif state == 2:
            #check if in turn mode
            if mode.get() == MODE_TURN:
                
                
                if initial_heading.get() != -1 and run_state.get() == 1:
                    #get all headings
                    initial_hdg = initial_heading.get()
                    current_hdg = heading.get()
                    
                    if initial_hdg < 180:
                        target_heading = initial_hdg + 180.0
                    else:
                        target_heading = initial_hdg - 180.0

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
                  
                    state = 3
        
        # State 3: Grid Turn 1
        elif state == 3:
            #if the run_state is off, stop the motors
            if run_state.get() == 0:
                effort_L.put(0)
                effort_R.put(0)
            else:
                #confirm we are in turn mode
                if mode.get() != MODE_TURN:
                    effort_L.put(0)
                    effort_R.put(0)
                    state = 1  #go to idle if not
                
                #check if in turn mode
                if mode.get() == MODE_TURN:
                    
                    #get error based on headings
                    current_heading = heading.get()
        
                    error = target_heading - current_heading
                    
                    if error > 360:
                        error -= 360


                    #turn if not facing the right direction
                    if abs(error) > HEADING_TOLERANCE:
                        
                        if error > 0:
                            #counterclockwise
                            effort_L.put(TURN_SPEED)
                            effort_R.put(-TURN_SPEED)
                        elif error < 0:
                            #clockwise
                            effort_L.put(-TURN_SPEED)
                            effort_R.put(TURN_SPEED)
                        
                    #Romi is facing the right way, stop      
                    else:
                        effort_L.put(0)
                        effort_R.put(0)

                        #set the grid navigation mode
                        mode.put(MODE_GRID_NAV)
                        
                        turn_started = False
                        pos_L = encoder_L.get_position()
                        FINAL_POSITION=pos_L+4500   #defines ecoder position relative to cutrrent location

                        state = 4

        # State 4: Grid Drive 1
        elif state == 4:
            if run_state.get() == 0:
                effort_L.put(0)
                effort_R.put(0)
            else:
                #check were in grid nav mode
                if mode.get() != MODE_GRID_NAV:
                    mode.put(MODE_GRID_NAV)
                
                #update encoders and varaibles
                encoder_L.update()
                encoder_R.update()
                
                pos_L = encoder_L.get_position()
                pos_R = encoder_R.get_position()

                current = heading.get()

                error  = target_heading - current
                
                #adjust effort to stay aligned with heading
                steering = yaw.calculate(error)
                effort_L.put(STRAIGHT_NAV_SPEED + steering)
                effort_R.put(STRAIGHT_NAV_SPEED - steering)
              
                #stop when it reaches set ecoder distance
                if pos_L > FINAL_POSITION:
                    effort_L.put(0)
                    effort_R.put(0)

                    mode.put(MODE_TURN)
                    state = 5
        
        # State 5: Grid Turn 2
        elif state == 5: #Refer to State 4 Position Turn 1 for comments, only heading direction is altered.
                if run_state.get() == 0:
                    effort_L.put(0)
                    effort_R.put(0)
                else:
                    if mode.get() != MODE_TURN:
                        effort_L.put(0)
                        effort_R.put(0)
                        state = 1 
                    
                    if mode.get() == MODE_TURN:
                        current_heading = heading.get()
                        error = target_heading_2 - current_heading
                        
                        if error > 360:
                            error -= 360
                        
                        turn_started = False

                        if abs(error) > 2 and not turn_started:
                            
                            if error != 0:
                                effort_L.put(15)
                                effort_R.put(-15)

                            turn_started = True   
                        
                        else:
                            effort_L.put(0)
                            effort_R.put(0)

                            encoder_L.update()
                            pos_L = encoder_L.get_position()
                            FINAL_POSITION=pos_L+500    #defines ecoder position relative to cutrrent location
                            mode.put(MODE_GRID_NAV)
                        
                            state = 6
        # State 6: Grid Drive 2
        elif state == 6:
            #check if in grid navigation mode
            if mode.get() == MODE_GRID_NAV:
                if run_state.get() == 0:
                    effort_L.put(0)
                    effort_R.put(0)
                else:
                    encoder_L.update()
                    encoder_R.update()
                    
                    pos_L = encoder_L.get_position()
                    pos_R = encoder_R.get_position()

                    #set motors to go straight 
                    effort_L.put(STRAIGHT_NAV_SPEED)
                    effort_R.put(STRAIGHT_NAV_SPEED)
                    
                    #stop wehen you reach set encoder distance
                    if pos_L > FINAL_POSITION:
                        effort_L.put(0)
                        effort_R.put(0)
                        position_threshold.put(0)   #defines ecoder position relative to cutrrent location
                        mode.put(MODE_LINE_FOLLOW)  #return to line follow mode after grid section

        # State 7: Diamond Turn
        elif state == 7:    

            #get new target heading
            target_heading_3 =target_heading+90
            if target_heading_3 >= 360.0:
                target_heading_3 -= 360.0
            elif target_heading_3 < 0.0:
                target_heading_3 += 360.0

            #stop if not int run
            if run_state.get() == 0:
                effort_L.put(0)
                effort_R.put(0)
            else:
                #get error from heading
                current_heading = heading.get()   
                error = target_heading_3 - current_heading
                if error > 360:
                    error -= 360

                turn_started = False
                
                #turn if not facing the right way
                if abs(error) > 2 and not turn_started:
                    
                    if error != 0:
                    #clockwise
                        effort_L.put(15)
                        effort_R.put(-15)

                    turn_started = True
                
                #Romi is facing the right way, stop
                else:
                    effort_L.put(0)
                    effort_R.put(0)
                    turn_started = False

                    state = 8

        # State 8: Diamond Drive
        elif state == 8:
            encoder_L.update()
            encoder_R.update()
            
            pos_L = encoder_L.get_position()
            pos_R = encoder_R.get_position()
            
            #set motors to go straight 
            effort_L.put(20)
            effort_R.put(20)
            
            #stop wehen you reach set encoder distance
            if pos_L > FINAL_POSITION:
                effort_L.put(0)
                effort_R.put(0)

                position_threshold.put(0) 
                mode.put(MODE_LINE_FOLLOW)  #return to line follow mode after grid section
                state = 1
    
        yield state

def imu_task(shares):
    """
    Task to read orientation data from the BNO055 IMU.
    
    Args:
        shares: Tuple containing (heading, initial_heading, is_calibrated)
    """
    heading, initial_heading = shares
    
    #  Reset
    state = 0
    imu = None
    heading_value = 0.0
    
    while True:

        # State 0: Init
        if state == 0:
            try:
                i2c = I2C(1, I2C.CONTROLLER)
                i2c.init(I2C.CONTROLLER, baudrate=100000)
                imu = IMU(i2c)
                
                state = 1
                
            except Exception as e:
                pass
        
        # State 1: Read Heading
        elif state == 1:
            try:
                #read imu heading (0-360 degrees)
                heading_value = imu.get_heading()
                
                #store heading
                heading.put(heading_value)
                
                #set heading as the initial refrece if  first reading
                if initial_heading.get() == -1:
                    initial_heading.put(heading_value)
            
            except Exception as e:
                state = 0  
        
        yield state

if __name__ == "__main__":

    # Shares
    effort_L = task_share.Share('f', thread_protect=True, name="Effort_L")
    effort_R = task_share.Share('f', thread_protect=True, name="Effort_R")
    line_position = task_share.Share('f', thread_protect=True, name="Line_Position")
    heading = task_share.Share('f', thread_protect=True, name="Heading")
    initial_heading = task_share.Share('f', thread_protect=True, name="Initial_Heading")
    bump_state = task_share.Share('B', thread_protect=True, name="Bump_State")
    mode = task_share.Share('B', thread_protect=True, name="Mode")
    run_state = task_share.Share('B', thread_protect=True, name="Run_State")
    position_threshold = task_share.Share('i', thread_protect=True, name="Position_Threshold_1")

    # Init shared variables
    heading.put(0.0)
    initial_heading.put(-1.0)  
    position_threshold.put(0)
    mode.put(MODE_LINE_FOLLOW)
    run_state.put(0)

    # Tasks
    motor_task = cotask.Task(motor_task, name="Motor Task", priority=2, 
                             period=10, profile=True, trace=False,
                             shares=(effort_L, effort_R, run_state))
    
    line_task = cotask.Task(line_task, name="Line Task", priority=3, 
                           period=10, profile=True, trace=False,
                           shares=(line_position, mode, effort_L, effort_R, 
                                   bump_state, run_state, position_threshold))
    
    position_task = cotask.Task(position_task, name="Position Task", priority=4,
                               period=15, profile=True, trace=False,
                                shares=(heading, initial_heading, 
                                       position_threshold, effort_L, effort_R, run_state, mode)) 
    
    imu_task = cotask.Task(imu_task, name="IMU Task", priority=1, 
                          period=50, profile=True, trace=False,
                          shares=(heading, initial_heading))
    
    bump_task = cotask.Task(bump_task, name="Bump Task", priority=5, 
                            period=50, profile=True, trace=False,
                            shares=(bump_state, mode,
                                    effort_L, effort_R))
    
    cotask.task_list.append(motor_task)
    cotask.task_list.append(line_task)
    cotask.task_list.append(position_task)
    cotask.task_list.append(imu_task)
    cotask.task_list.append(bump_task)

    #memory garbage collector
    import gc
    gc.collect()
   
    #define user button
    user_button = Pin('C13', Pin.IN, Pin.PULL_DOWN)
    
    #start in line following mode
    mode.put(MODE_LINE_FOLLOW)

    #reset heading
    heading.put(0.0)
    initial_heading.put(-1.0)
    
    #User Button as Kill Switch
    last_button_state = 0
    
    while True:

        try:
        #check if button is pressed
            button_state = user_button.value()
            if button_state == 1 and last_button_state == 0:
                if run_state.get() == 0:
                    button_state = ()
                    run_state.put(1)
                else:
                    run_state.put(0)
                    
            last_button_state = button_state
            
            #run the scheduler
            cotask.task_list.pri_sched()
        
        except KeyboardInterrupt:
            break

