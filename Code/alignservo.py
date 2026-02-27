from myservo import Servo
import time

servo=Servo(9)
servo.ServoAngle(0)
time.sleep_ms(1000)

servo.deinit()