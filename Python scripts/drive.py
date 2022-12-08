#!/usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep


drive_f = 21
drive_b = 20

turn_r = 16
turn_l = 26

GPIO.setwarnings(False)	

GPIO.setmode(GPIO.BCM) # Choose BCM to use GPIO numbers instead of pin numbers (BOARD)

GPIO.setup(drive_f,GPIO.OUT)
GPIO.setup(drive_b,GPIO.OUT)
GPIO.setup(turn_r,GPIO.OUT)
GPIO.setup(turn_l,GPIO.OUT)

while True:
    val = input("Enter command: ")
    
    if val == "d":
        GPIO.output(drive_f,1)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,0)
        sleep(1)

    elif val == "b":
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,1)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,0)
        sleep(1)
        
    elif val == "s":
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,0)
        sleep(1)
        
    elif val == "r":
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,1)
        GPIO.output(turn_l,0)
        sleep(1)
        
    elif val == "l":
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,1)
        sleep(1)
        
        
    elif val == "dr":
        GPIO.output(drive_f,1)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,1)
        GPIO.output(turn_l,0)
        sleep(1)
            
    elif val == "dl":
        GPIO.output(drive_f,1)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,1)
        sleep(1)
        
    elif val == "br":
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,1)
        GPIO.output(turn_r,1)
        GPIO.output(turn_l,0)
        sleep(1)
            
    elif val == "bl":
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,1)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,1)
        sleep(1)     
    
    else:
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,0)
        sleep(1)
    
    
    #time.sleep(.25)
    #GPIO.output(pin,False)
    #time.sleep(.25)
        
    
GPIO.cleanup()
