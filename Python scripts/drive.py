#!/usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep


drive_f = 21 #drive
drive_b = 20 #reverse

turn_r = 16 #turn right
turn_l = 26 #turn left

GPIO.setwarnings(False)	

GPIO.setmode(GPIO.BCM) # Choose BCM to use GPIO numbers instead of pin numbers (BOARD)

GPIO.setup(drive_f,GPIO.OUT)
GPIO.setup(drive_b,GPIO.OUT)
GPIO.setup(turn_r,GPIO.OUT)
GPIO.setup(turn_l,GPIO.OUT)

while True:
    val = input("Enter command: ")
    
    if val == "d": #drive
        GPIO.output(drive_f,1)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,0)
        sleep(1)

    elif val == "b": #reverse
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,1)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,0)
        sleep(1)
        
    elif val == "s": #stop
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,0)
        sleep(1)
        
    elif val == "r": #turn right
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,1)
        GPIO.output(turn_l,0)
        sleep(1)
        
    elif val == "l": #turn left
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,1)
        sleep(1)
        
        
    elif val == "dr": #drive and turn right
        GPIO.output(drive_f,1)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,1)
        GPIO.output(turn_l,0)
        sleep(1)
            
    elif val == "dl": #drive and turn left
        GPIO.output(drive_f,1)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,1)
        sleep(1)
        
    elif val == "br": #reverse and turn right
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,1)
        GPIO.output(turn_r,1)
        GPIO.output(turn_l,0)
        sleep(1)
            
    elif val == "bl": #reverse and turn left
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,1)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,1)
        sleep(1)     
    
    else:   #default stops
        GPIO.output(drive_f,0)
        GPIO.output(drive_b,0)
        GPIO.output(turn_r,0)
        GPIO.output(turn_l,0)
        sleep(1)
    
    
    
GPIO.cleanup()
