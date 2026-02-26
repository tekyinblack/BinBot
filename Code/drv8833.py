from machine import PWM, Pin
import utime

class drv8833:


    def __init__(self, left_fwd_pin=15, left_rev_pin=14, right_fwd_pin=12, right_rev_pin=11, frequency=2000):
                 
        self.frequency = frequency

        self.leftFwd = PWM(Pin(left_fwd_pin))
        self.leftFwd.freq(frequency)
        #self.leftFwd.duty_u16(0)
        self.leftRev = PWM(Pin(left_rev_pin))
        self.leftRev.freq(frequency)
        #self.leftRev.duty_u16(0)
        self.rightFwd = PWM(Pin(right_fwd_pin))
        self.rightFwd.freq(frequency)
        #self.rightFwd.duty_u16(0)
        self.rightRev = PWM(Pin(right_rev_pin))
        self.rightRev.freq(frequency)
        #self.rightRev.duty_u16(0)
   
    #Driving the motor is simpler than the servo - just convert 0-100% to 0-4095 and push it to the correct registers.
    #each motor has 4 writes - low and high bytes for a pair of registers. 
    def motorOn(self,motor, speed, direction):
        #cap speed to 0-100%
        if (speed<0):
            speed = 0
        elif (speed>100):
            speed=100
        #convert 0-100 to 0-65535
        PWM = int(speed*655.35)
        if motor == "l":
            if direction == "f":
                self.leftFwd.duty_u16(PWM)
                self.leftRev.duty_u16(0)
            elif direction == "r":
                self.leftFwd.duty_u16(0)
                self.leftRev.duty_u16(PWM)
            else:
                raise Exception("INVALID DIRECTION") #harsh, but at least you'll know
        elif motor == "r":
            if direction == "f":
                self.rightFwd.duty_u16(PWM)
                self.rightRev.duty_u16(0)
            elif direction == "r":
                self.rightFwd.duty_u16(0)
                self.rightRev.duty_u16(PWM)
            else:
                raise Exception("INVALID DIRECTION") #harsh, but at least you'll know
        else:
            raise Exception("INVALID MOTOR") #harsh, but at least you'll know
    #To turn off set the speed to 0...
    def motorOff(self,motor):
        self.motorOn(motor,0, "f")
        


         

