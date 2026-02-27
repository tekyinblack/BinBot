# binbot web server by tekyinblack
# based on Pico W documentation 
# v1 converted from wheelchair code and micromouse driver examples
import network
import asyncio
import socket
import time
from machine import Pin
import _thread
from bin_web_1 import webpages
import gc
import rp2
import drv8833
import utime
from myservo import Servo

#setup lid servo
lid=Servo(13)
lid.ServoAngle(90)
time.sleep_ms(1000)

# line follower parameters
LINE_MIDDLE = 3   # The middle sensor position
PROPORTION = 10.0  # Proportional Control constant (fine-tune as needed)
SPEED = 70       # Preset motor speed for straight movement
spin_speed = 35
compensation = 1.05
LOOP_LIMIT = 5000
timeshift = 0.0
Kp = 13   #13.0   # proportional constant
Ki = 0  #0.65  #0.0    # integral constant
Kd = 6.3 #10.0   #65.0  #0.0    # derivative constant

top_sensor = False
bottom_sensor = False

newspeed1 = 0.0
newspeed2 = 0.0
addspeed = 0.0
record = False

motor = drv8833.drv8833(15,14,11,12)



state = "FOLLOW"

#<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>
# line follower routines

        
# <<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



# initial sleep to give time to reset pico before threads start
# avoids accidental lockouts
print ("Waiting to start...")
time.sleep(5)
print ("Starting")



def forward():
    motor.motorOn(1,70,'f')
    motor.motorOn(2,70,'f')

    
def stop():
    
    motor.motorOff(1)
    motor.motorOff(2)
    
def reverse():
    motor.motorOn(1,70,'r')
    motor.motorOn(2,70,'r')

def motors():
    
    while True:

        motor.motorOn(1,70,'f')
        motor.motorOn(2,70,'f')
        time.sleep(5)
        motor.motorOn(1,0,'f')
        motor.motorOn(2,70,'f')    
        time.sleep(5)
        motor.motorOn(1,70,'f')
        motor.motorOn(2,0,'f')
        time.sleep(5)




try:
    # Run the event loop indefinitely
    motors()
except Exception as e:
    print('Error occured: ', e)
except KeyboardInterrupt:
    print('Program Interrupted by the user')
finally:
    finished = True

    time.sleep(0.01)
    # float the pins
    motor.motorOff(1)
    motor.motorOff(2)
    lid.ServoAngle(90)

    print ("Program waiting to end")
    # hang around to let threads end
    time.sleep(1)
