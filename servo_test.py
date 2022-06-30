import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.OUT)
servo_top = GPIO.PWM(17,50)
servo_top.start(2.5)
#GPIO.setup(17, GPIO.OUT)
#servo_bot = GPIO.PWM(17,50)
GPIO.setup(25, GPIO.IN) # button 1
GPIO.setup(24, GPIO.IN) # button 2
#GPIO.setup(23, GPIO.IN) # button 3
#GPIO.setup(22, GPIO.IN) # button 4

while True:
    if GPIO.input(25)== False: # button 1 is pressed
            print ('pressed')
            time.sleep(0.3)