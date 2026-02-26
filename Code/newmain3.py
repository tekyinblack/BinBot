# binbot web server by tekyinblack
# based on Pico W documentation 
# v1 converted from wheelchair code and micromouse driver examples
# v3 first complete feature code
#    still have problems with Left motor drive so have introduced a compensation for it

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
from hcsr04 import HCSR04

#setup lid servo
lid=Servo(9)
lid.ServoAngle(90)
bin_closed = True
time.sleep_ms(1000)

# line follower parameters
LINE_MIDDLE = 3   # The middle sensor position
PROPORTION = 10.0  # Proportional Control constant (fine-tune as needed)
SPEED = 70       # Preset motor speed for straight movement

compensation = 1.05

Kp = 13   #13.0   # proportional constant
Ki = 0  #0.65  #0.0    # integral constant
Kd = 6.3 #10.0   #65.0  #0.0    # derivative constant

rubbish = HCSR04(trigger_pin=8, echo_pin=10, echo_timeout_us=20000)
collision = HCSR04(trigger_pin=2, echo_pin=5, echo_timeout_us=20000)

collision_range = 100
rubbish_range = 200
collision_delay = 2
bin_open_delay = 5
bin_open_timer = time.time()
following_timer = time.time()

following = True


newspeed1 = 0.0
newspeed2 = 0.0
addspeed = 0.0
integral = 0.0
lasterror = 0.0
derivative = 0.0
error = 0.0
lastspeed1 = 0
lastspeed2 = 0
actual = 0



motor = drv8833.drv8833(15,14,11,12)

# line sensor values
line_sensor = [
    Pin(22, Pin.IN),
    Pin(21, Pin.IN),
    Pin(20, Pin.IN),
    Pin(19, Pin.IN),
    Pin(18, Pin.IN)
]




#<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>
# line follower routines
def get_average_line_position():
    outputs = []
    for n, sensor in enumerate(line_sensor):
        if sensor.value():
            outputs.append(n + 1)
    if outputs:
        return sum(outputs)/len(outputs)
    else:
        return 0
    
def get_error_line_position():
    outputs = []
    for n, sensor in enumerate(line_sensor):
        if sensor.value():
            outputs.append(n + 1)
    if outputs:
        return sum(outputs)/len(outputs),len(outputs)
    else:
        return 0
        
# <<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



# initial sleep to give time to reset pico before threads start
# avoids accidental lockouts
print ("Waiting to start...")
time.sleep(5)
print ("Starting")


# onboard indicator led to show activity
#led_indicator = machine.Pin("LED", machine.Pin.OUT)

# Initialize variables


def forward():
    motor.motorOn("l",70,'f')
    motor.motorOn("r",70,'f')

    
def stop():
    global lastspeed1, lastspeed2
    print("Motor stop")
    motor.motorOff("l")
    motor.motorOff("r")
    lastspeed1 = 0
    lastspeed2 = 0
    
def reverse():
    motor.motorOn("l",70,'r')
    motor.motorOn("r",70,'r')
    
def left():
    motor.motorOn("l",35,'f')
    motor.motorOn("r",35,'r')
    
def right():
    motor.motorOn("l",35,'r')
    motor.motorOn("r",35,'f')

def lid_open():
    # open bin lid
    global bin_closed
    print ("Opening lid")
    #lid.ServoAngle(170)
    for i in range(90, 170, 1):
        lid.ServoAngle(i)
        time.sleep_ms(15)
    time.sleep(1)
    bin_closed = False

def lid_close():
    # close bin lid
    global bin_closed
    print ("Closing lid")
    #lid.ServoAngle(90)
    for i in range(170, 90, -1):
        lid.ServoAngle(i)
        time.sleep_ms(15)
    time.sleep(1)
    bin_closed = True
    


def detect_collision():
    global following
    global following_timer
    global collision_delay
    if following:
        # run the bottom ultrasonic and wait
        if collision.distance_mm() < collision_range:
            # if detected
            print ("Collision detected")
            #    stop motors
            stop()
            #    set following to False
            following = False
            #    set following timer
            
            following_timer = time.time() + collision_delay
            print (following_timer)
    pass

def check_following_timer():
    # if following timer has expired
    #     set following  to True
    global following, following_timer
    if not following:
        if time.time() > following_timer:
            following = True
            print("Following")
        
    pass

def detect_rubbish():
    # if the bin_closed is True
    global bin_closed
    global bin_open_timer
    global bin_open_delay
    if bin_closed:
        # run the top ultrasonic and wait
        if rubbish.distance_mm() < rubbish_range:
        # if detected
            print ("Rubbish detected")
        #      stop motors
            stop()
            #      set bin_closed to False
            #      open bin
            lid_open()
            #      set bin open timer
            bin_open_timer = time.time() + bin_open_delay
    pass

def check_bin_open_timer():
    # if bin closed is false
    global bin_closed, bin_open_timer
    if not bin_closed:
        if time.time() > bin_open_timer:
            lid_close()
    #     is bin_open_timer expired
    #           close bin
    #           set bin_closed to True
    pass


def line_follower():
    global integralfactor
    global SPEED
    global integral 
    global lasterror 
    global derivative 
    global error 
    global lastspeed1
    global lastspeed2
    global following, bin_closed
    if (following and bin_closed):
        # this code only runs when bin is closed and following set to true, when there is no obstructions detected
        # Calculate the error value based on the difference between the line position and expected position
        #  if line_sensor

        actual = get_average_line_position()
        if actual != 0:
            # on line, continue pid
            error = LINE_MIDDLE - actual
            #integral = integralfactor * integral + error
            integral = integral + error
            derivative = lasterror - error
            
            lasterror=error
            addspeed = Kp * error + Ki * integral + Kd * derivative
            # Apply the Proportional Control constant to the error to get the correction value
            # correction = error * PROPORTION
            #print(loop_count)
            newspeed1 = SPEED + addspeed
            newspeed2 = SPEED - addspeed

            if newspeed1 >= SPEED:
                newspeed2 = newspeed2 - (newspeed1-SPEED)
                newspeed1 = SPEED
            elif newspeed1 < 0:
                newspeed1 = 0
       
            
            if newspeed2 >= SPEED:
                newspeed1 = newspeed1 - (newspeed2-SPEED)
                newspeed2 = SPEED
            elif newspeed2 < 0:
                newspeed2 = 0
            if (abs(lastspeed1 - newspeed1) >= newspeed1 * 0.1) or (abs(lastspeed2 - newspeed2) >= newspeed2 * 0.1):

                motor.motorOn("l",newspeed1,'f')
                motor.motorOn("r",newspeed2,'f')
                lastspeed1 = newspeed1
                lastspeed2 = newspeed2
        else:
            # off line, start error recovery
            # stop!
            newspeed1 = 0
            newspeed2 = 0
            motor.motorOff("l")
            motor.motorOff("r")

            # get line position again just in case we stopped on a line
            actual = get_average_line_position()
            
            # zero the old PID
            integral = 0.0
            lasterror = 0.0
            derivative = 0.0
            error = 0.0
            
            # spin until line found
 #<<<<<<<<<<<<<<< Need a solution for the line search and not just a loop           
            while (actual == 0): # and width > 3):

                if  newspeed1 != SPEED/2:
                    newspeed1 = SPEED/2
                    newspeed2 = SPEED/2
                    motor.motorOn("l",newspeed1,'r')
                    motor.motorOn("r",newspeed2,'f')
                actual = get_average_line_position()

                time.sleep(0.04)         

# run line follower
   # line follower only runs if following set to true and bin_closed set to true
# detect bottom ultrasonic
   # if detected,
       # set following to stop
       # stop motors
       # set following timer
# has following timer expired
   # set following to go

      
# run line follower
# detect top ultrasonic
   # if detected,
       # set bin_closed to false
       # stop motors
       # open bin
       # start open bin timer
# is bin_closed false
    # is bin open timer expired
         # if timer expired
             # close bin
             # set bin_closed to true


#try:
    # Run the event loop indefinitely
    

while True:
    line_follower()
    detect_collision()
    check_following_timer()
    line_follower()
    detect_rubbish()
    check_bin_open_timer()
    
#except Exception as e:
#    print('Error occured: ', e)
#except KeyboardInterrupt:
#    print('Program Interrupted by the user')
#finally:
    

time.sleep(0.01)
# float the pins
motor.motorOff("l")
motor.motorOff("r")
lid.ServoAngle(90)

print ("Program waiting to end")
# hang around to let threads end
time.sleep(1)
