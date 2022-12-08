#!/usr/bin/env python3

import RPi.GPIO as GPIO
import tkinter
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

master =tkinter.Tk()
master.title("grid() method")
master.geometry("350x275")

def straightLeft():
    GPIO.output(drive_f,1)
    GPIO.output(drive_b,0)
    GPIO.output(turn_r,0)
    GPIO.output(turn_l,1)
    sleep(1)

def straight():
    GPIO.output(drive_f,1)
    GPIO.output(drive_b,0)
    GPIO.output(turn_r,0)
    GPIO.output(turn_l,0)
    sleep(1)

def straightRight():
    GPIO.output(drive_f,1)
    GPIO.output(drive_b,0)
    GPIO.output(turn_r,1)
    GPIO.output(turn_l,0)
    sleep(1)
    
def Left():
    GPIO.output(drive_f,0)
    GPIO.output(drive_b,0)
    GPIO.output(turn_r,0)
    GPIO.output(turn_l,1)
    sleep(1)
    
def Stop():
    GPIO.output(drive_f,0)
    GPIO.output(drive_b,0)
    GPIO.output(turn_r,0)
    GPIO.output(turn_l,0)
    sleep(1) 

def Right():
    GPIO.output(drive_f,0)
    GPIO.output(drive_b,0)
    GPIO.output(turn_r,1)
    GPIO.output(turn_l,0)
    sleep(1)
    
def backLeft():
    GPIO.output(drive_f,0)
    GPIO.output(drive_b,1)
    GPIO.output(turn_r,0)
    GPIO.output(turn_l,1)
    sleep(1)   

def back():
    GPIO.output(drive_f,0)
    GPIO.output(drive_b,1)
    GPIO.output(turn_r,0)
    GPIO.output(turn_l,0)
    sleep(1)    
    
def backRight():
    GPIO.output(drive_f,0)
    GPIO.output(drive_b,1)
    GPIO.output(turn_r,1)
    GPIO.output(turn_l,0)
    sleep(1)   
        

    
straightLeftButton=tkinter.Button(master, text=("Straight Left"), command = straightLeft)
straightLeftButton.grid(row=1,column=0)

straightButton=tkinter.Button(master, text=("Straight"), command = straight)
straightButton.grid(row=1,column=1)

straightRightButton=tkinter.Button(master, text=("Straight Right"), command = straightRight)
straightRightButton.grid(row=1,column=2)

leftButton=tkinter.Button(master, text=("Left"), command = Left)
leftButton.grid(row=2,column=0)

stopButton=tkinter.Button(master, text=("Stop"), command = Stop)
stopButton.grid(row=2,column=1)

rightButton=tkinter.Button(master, text=("Right"), command = Right)
rightButton.grid(row=2,column=2)

backLeftButton=tkinter.Button(master, text=("Back Left"), command = backLeft)
backLeftButton.grid(row=3,column=0)

backButton=tkinter.Button(master, text=("Back"), command = back)
backButton.grid(row=3,column=1)

backRightButton=tkinter.Button(master, text=("Back Right"), command = backRight)
backRightButton.grid(row=3,column=2)

master.mainloop()
GPIO.cleanup()
