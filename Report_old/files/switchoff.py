mport RPi.GPIO as GPIO
import os
import time

#set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)
GPIO.setup(10,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#function called on pin interrupt
def button_triggered(channel):
    counter = 0
    counter_on = 0
    while (counter <= 6):
        time.sleep(1)
        counter+=1
        if (GPIO.input(10)):
            counter_on+=1
        if (counter_on >= 3):
            break

    if (counter_on >= 3):
        print("switchoff.py: Raspberry shutting down now")
        os.system("sudo halt")
    elif (counter_on < 3):
        print("switchoff.py: Rapsberry is going to reboot now")
        os.system("sudo reboot")
#setup pin interrupt
GPIO.add_event_detect(10,GPIO.RISING,callback=button_triggered,bouncetime=300)

#wait forever
while True:
    time.sleep(0.001)

GPIO.cleanup()

