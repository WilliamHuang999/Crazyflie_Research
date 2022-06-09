import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18,GPIO.OUT)
servo_top = GPIO.PWM(18,50)
GPIO.setup(17, GPIO.OUT)
servo_bot = GPIO.PWM(17,50)
GPIO.setup(25, GPIO.IN) # button 1
GPIO.setup(24, GPIO.IN) # button 2
GPIO.setup(23, GPIO.IN) # button 3
GPIO.setup(22, GPIO.IN) # button 4
servo_top.start(2.5)
servo_bot.start(2.5)
servo_top.ChangeDutyCycle(0)
servo_bot.ChangeDutyCycle(0)

#duty cycle 2 is 0 degrees
#duty cycle 12 is 180 degrees
duty1 = 2
duty2 = 2

try:
    while True:
        if GPIO.input(25)== False: # button 1 is pressed
            print ('pressed')
            if duty1 <= 12: # move the servo
                servo_top.ChangeDutyCycle(duty1)
                time.sleep(0.3)
                servo_top.ChangeDutyCycle(0)
                duty1 = duty1+1
        if GPIO.input(24)== False: #button 2 pressed
            print ('pressed')
            duty1 = 2
            servo_top.ChangeDutyCycle(duty1)
            time.sleep(0.3)
            servo_top.ChangeDutyCycle(0)
        if GPIO.input(23)== False: # button 3 is pressed
            print ('pressed')
            if duty2 <= 12: # move the servo
                servo_bot.ChangeDutyCycle(duty2)
                time.sleep(0.3)
                servo_bot.ChangeDutyCycle(0)
                duty2 = duty2+1
        if GPIO.input(22)== False: # button 4 is pressed 
            print ('pressed')
            duty2 = 2
            servo_bot.ChangeDutyCycle(duty1)
            time.sleep(0.3)
            servo_bot.ChangeDutyCycle(0)
except KeyboardInterrupt:
    servo_top.stop()
    GPIO.cleanup()