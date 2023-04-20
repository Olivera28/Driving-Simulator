# Author: Abdullahnayeem Mizan
# Date  : 4/20/2023

#Dashboard GUI the represents the gear and velocity of the RC Car
 
from tkinter import *
import tkinter as tk
import datetime
import tk_tools
import serial

root = tk.Tk()
root.title('DashBoard')
root.attributes('-fullscreen',True)
root.configure(bg='orange')

speed = tk_tools.Gauge(root, width=918, height=800,unit = "m/s", min_value=0.0, max_value=100.0, label='speed', divisions=10, yellow=50, red=80, yellow_low=0, red_low=0,
bg='orange')
speed.place(x=0,y=0)

drive = tk_tools.Led(root, size = 200,bg = 'orange')
drive.place(x=999,y=33)
drive.to_grey()

park = tk_tools.Led(root, size = 200,bg = 'orange')
park.place(x=999,y=266)
park.to_grey()

reverse = tk_tools.Led(root, size = 200,bg = 'orange')
reverse.place(x=999,y=499)
reverse.to_grey()

drive_label = tk.Label(root, text="P", font=("Comic Sans",50), bg="orange")
drive_label.place(x=945, y=133)

park_label = tk.Label(root, text="R",font=("Comic Sans",50), bg="orange")
park_label.place(x=945, y=366)

reverse_label = tk.Label(root, text="D",font=("Comic Sans",50), bg="orange")
reverse_label.place(x=945, y=599)

def init_serial():
    global ser          
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = "/dev/ttyACM0"   #COM Port Name Start from 0
    #Specify the TimeOut in seconds, so that SerialPort
    #Doesn't hangs
    ser.timeout = 10
    ser.open()          #Opens SerialPort
    # print port open or closed
    if ser.isOpen():
        print("Open: " + ser.portstr)
        
init_serial()
def clock():
    global value
    bytes = ser.readline()  #Read from Serial Port
    #print ("You sent: ")      
    #value = int(bytes)
    #print (bytes)          #Print What is Read from Port
    parseData(bytes);
    #speed.set_value(value)
    #acceleration.set_value(value)
    #set_gear()
    
    root.after(1, clock) # run itself again after 10 ms

def set_gear(gear):
    #print(gear)
    if (gear == 'p'):
        drive.to_green(on=True)
        park.to_grey()
        reverse.to_grey()
    if (gear == 'r'):
        drive.to_grey()
        reverse.to_grey()
        park.to_green(on=True)
    if (gear == 'd'):
        park.to_grey()
        drive.to_grey()
        reverse.to_green(on=True)

def parseData(enter):
    parse = enter.decode("utf-8")
    #print(parse)
    mystring =parse.split(",")
    print(mystring)
    set_gear(mystring[0])
    speed.set_value(mystring[1])
    
    
clock()

root.mainloop()
