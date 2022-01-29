import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
TOUCH_PIN = 22

GPIO.setup(TOUCH_PIN, GPIO.IN)
lOld = not GPIO.input(TOUCH_PIN)
print('Starting up the LIGHT Module (click on STOP to exit)')
time.sleep(0.5)
while True:
  if GPIO.input(TOUCH_PIN) != lOld:
    if (GPIO.input(TOUCH_PIN)):
      print ('Press')
    else:
      print ('not press') 

  lOld = GPIO.input(TOUCH_PIN)
  time.sleep(0.2)