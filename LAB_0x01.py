# main.py -- put your code here!
from pyb import Pin, Timer, delay
#mport time import sleep_ms

"""Set motor control pins"""
# left motor
Motor_L_Ena = Pin(pyb.Pin.cpu.D4, Pin.OUT, Pin.PULL_DOWN)
Motor_L_Ena.value(0)
Motor_L_Dir = Pin('D5', Pin.OUT, Pin.PULL_DOWN)
# right motor
Motor_R_Ena = Pin('C14', Pin.OUT, Pin.PULL_DOWN)
Motor_R_Ena.value(0)
Motor_R_Dir = Pin('C13', Pin.OUT, Pin.PULL_DOWN)

"""Initialize motor PWM"""
# Left motor
tim2 = Timer(2, freq=1000)
Motor_L_PWM = tim2.channel(3, pin= Pin('D6'), mode=Timer.PWM, pulse_width_percent=0)
# Right motor
tim4 = Timer(4, freq=1000)
Motor_R_PWM = tim4.channel(2, pin= Pin('B7'), mode=Timer.PWM, pulse_width_percent=0)

"""Initialize timers for encoder counting"""
# left motor
encoder_L = Timer(3, period = 0xFFFF, prescaler = 0)
encoder_L.channel(1, pin=Pin('D12'), mode=Timer.ENC_AB)
encoder_L.channel(2, pin=Pin('D11'), mode=Timer.ENC_AB)
#right motor
encoder_R = Timer(1, period = 0xFFFF, prescaler = 0)
encoder_R.channel(1, pin=Pin('D7'), mode=Timer.ENC_AB)
encoder_R.channel(2, pin=Pin('D8'), mode=Timer.ENC_AB)

"""Read left and right encoders"""
def read_encoder():
    count_L = encoder_L.counter()
    count_R = encoder_R.counter()
    print(f"{count_L}, {count_R}")
    return count_L, count_R

"""Motor testing sequence"""
def main():

    # set motor duty cycles
    Motor_L_PWM.pulse_width_percent(50)
    Motor_R_PWM.pulse_width_percent(50)

    # enable both motors
    Motor_L_Ena.value(1)
    Motor_R_Ena.value(1)

    # set motors to forward direction
    Motor_L_Dir.value(0)
    Motor_R_Dir.value(0)

    # read encoder every 100ms for a total of 2.5sec
    for i in range(25):
        read_encoder()
        delay(100)
    
    # set motors to backward direction
    Motor_L_Dir.value(1)
    Motor_R_Dir.value(1)

    for i in range(25):
        read_encoder()
        delay(100)

    # change motor duty cycle
    Motor_L_PWM.pulse_width_percent(10)
    Motor_R_PWM.pulse_width_percent(10)

    for i in range(25):
        read_encoder()
        delay(100)

    Motor_L_PWM.pulse_width_percent(0)
    Motor_R_PWM.pulse_width_percent(0)

if __name__ == "__main__":
     main()

