import time
import RPi.GPIO as GPIO
 
# define pin number PQ.06
output_pin = 31
 
# set pin as BOARD mode
GPIO.setmode(GPIO.BOARD)
 
# set pin mode
GPIO.setup(output_pin, GPIO.OUT)
 
print("Press CTRL+C to exit")
curr_value = GPIO.HIGH
try:
    while True:
        time.sleep(0.07)
        print("pin {} now is {}".format(output_pin, curr_value))
        GPIO.output(output_pin, curr_value)
        # blink
        curr_value ^= GPIO.HIGH
finally:
    GPIO.cleanup()
