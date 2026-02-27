from machine import Pin
import time
from hcsr04 import HCSR04

sensor = HCSR04(trigger_pin=8, echo_pin=10, echo_timeout_us=30000)
sensor2 = HCSR04(trigger_pin=2, echo_pin=5, echo_timeout_us=30000)
while True:
    try:
        distance_mm = sensor.distance_mm()
        print('Distance: {} mm'.format(distance_mm))
        time.sleep(.05)
        distance_mm = sensor2.distance_mm()
        print('Distance2: {} mm'.format(distance_mm))

    except OSError as e:
        print('Error:', e)
    
    time.sleep(1) 