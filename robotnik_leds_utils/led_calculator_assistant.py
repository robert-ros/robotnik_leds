#!/usr/bin/env python3

# sudo apt-get install python3.8 python3-tk

from tkinter import *
from tkinter import scrolledtext
import math


# This value depends on refresh time used by ALS module. By default is 20 ms
refresh_time = 20

window = Tk()

# Init led
# End led
# Desired Time 

# Init led 
labelInitLed = Label(text="Init led")
labelInitLed.grid(column=1, row=1, sticky=W, padx=(10, 20), pady = (5,0))

entryInitLed = Entry(width=10)
entryInitLed.grid(column=1, row=2, padx=(10, 20))
entryInitLed.insert(END, '1')

# End led
labelEndLed = Label(text="End led")
labelEndLed.grid(column=1, row=3, sticky=W, padx=(10, 20), pady = (5,0))

entryEndLed = Entry(width=10)
entryEndLed.grid(column=1, row=4, padx=(10, 20))
entryEndLed.insert(END, '140')

# Desired time
labelDesiredTime = Label(text="Desired time")
labelDesiredTime.grid(column=1, row=5, sticky=W, padx=(10, 20), pady = (5,0))

entryDesiredTime = Entry(width=10)
entryDesiredTime.grid(column=1, row=6, padx=(10, 20))
entryDesiredTime.insert(END, '1000')

# Led increment
labelLedIncrement = Label(text="Led increment")
labelLedIncrement.grid(column=1, row=7, sticky=W, padx=(10, 20), pady = (5,0))

entryLedIncrement = Entry(width=10)
entryLedIncrement.grid(column=1, row=8, padx=(10, 20))
entryLedIncrement.insert(END, '1')


# Output text
#text = Text(window)
text = scrolledtext.ScrolledText(window,width=40,height=10)
text.place(x = 110, y = 10, width=300, height=160)

def findSpeed(start_led, end_led, led_increment, speed):

    global refresh_time

    number_of_leds = end_led - start_led + 1

    number_of_leds = math.ceil(number_of_leds/led_increment)

    # When all leds turn off to repeat the secuence. This is a new extra led 
    number_of_leds_in_system = number_of_leds + 1

    speed_per_pixel = speed / number_of_leds_in_system   

    # Buscar el multiplo de 20 mas cercano a la baja
    remainder = speed_per_pixel % refresh_time

    # Si el resto es inferior al 60% de refresh_time, redondeo a la baja,  si no, redondeo a la alta
    if remainder < refresh_time*0.6:
        speed_per_pixel_rounded = speed_per_pixel - remainder
        
    else:
        inverse_remainder = refresh_time - remainder
        speed_per_pixel_rounded = speed_per_pixel + inverse_remainder


    # Si speed_per_pixe_rounded se ha redondeado a cero, automaticamente, pasa a valer el tiempo minimo
    # disponible, es decir, refresh_time

    if speed_per_pixel_rounded > 0:
        new_speed_per_pixel = speed_per_pixel_rounded
    else:
        new_speed_per_pixel = refresh_time

    new_speed = new_speed_per_pixel * number_of_leds_in_system

    return new_speed
    
def calculateAvailableIntervals():

    start_led = int(entryInitLed.get())
    end_led = int(entryEndLed.get())
    led_increment = int(entryLedIncrement.get())
    desiredSpeed = int(entryDesiredTime.get())

    # Clear text output
    text.delete('1.0', END)
    

    text.insert(END, "You must use one of the following led " + '\n' +
                    "intervals if you want to achieve a real time of " 
                         + str(desiredSpeed) + " ms in shift effect:" +  '\n' + '\n')
        
    findAnyResult = False

    for led in range(start_led, end_led+1):

        result = findRealTime(start_led, led, led_increment, desiredSpeed)

        if result == desiredSpeed:
            findAnyResult = True
            text.insert(END, "[" + str(start_led) +  ", " + str(led) + "]" + '\n')

    if not findAnyResult:

        text.insert(END, "No combination reaches the desired time" + '\n' + 
        "Consider modifying the conditions" + '\n')

def roundToRefreshTime(speed_decimal):

    global refresh_time
    speed_rounded = 0
    speed = int(speed_decimal)

    remainder = speed % refresh_time

    # Round up
    if(remainder < refresh_time*0.6):

        speed_rounded = speed - remainder
    

    # Else, rounds down
    else:

        inverse_remainder = refresh_time - remainder
        speed_rounded = int(speed + inverse_remainder)
    


    #If speed_rounded has been rounded to zero, then it is assigned the minimum possible value, 
    # that is, refresh_time

    if(speed_rounded <= 0):

          speed_rounded = refresh_time
    

    return speed_rounded
    
    

def findRealTime(start_led, end_led, led_increment, speed):

    global refresh_time 
 
    number_of_leds = end_led - start_led +1
   
    number_of_leds_in_system = math.ceil(number_of_leds/led_increment)
    
    speed_per_led_raw = speed / number_of_leds_in_system

    
    real_speed_per_led = roundToRefreshTime(speed_per_led_raw)

    real_speed = real_speed_per_led * number_of_leds_in_system

    return real_speed




def calculateRealTime():

    start_led = int(entryInitLed.get())
    end_led = int(entryEndLed.get())
    led_increment = int(entryLedIncrement.get())
    speed = int(entryDesiredTime.get())

    # Clear text output
    text.delete('1.0', END)

    real_speed = findRealTime(start_led, end_led, led_increment, speed)


    text.insert(END, "If you want to use the LED interval:" + '\n' + '\n' +
                     "[" + str(start_led) + ", " + str(end_led) + "] " + '\n' + '\n' +
                    "The real assigned speed in shift effect will be:"  +  '\n' +  '\n' +
                     str(real_speed) + " ms " +  '\n' + '\n')

    

    return real_speed



def calculate():


    if getSelected.get() == 1:

        calculateAvailableIntervals()

    if getSelected.get() == 2:

        calculateRealTime()




getSelected = IntVar()
radInterval = Radiobutton(text="Get interval", value=1, variable= getSelected)
radInterval.place(x = 120, y = 180)
getSelected.set(1)

realtime = Radiobutton(text="Get real speed", value=2, variable= getSelected)
realtime.place(x = 220, y = 180)


# Calculate button
calculateButton = Button(window, text="Calculate", command=calculate)
calculateButton.grid(column=1,row=9, padx=(10, 20), pady = (5,0))


w = 425 # width for the Tk root
h = 220 # height for the Tk root

# get screen width and height
ws = window.winfo_screenwidth() # width of the screen
hs = window.winfo_screenheight() # height of the screen

# calculate x and y coordinates for the Tk root window
x = (ws/2) - (w/2)
y = -100 + ((hs/2) - (h/2))


# set the dimensions of the screen and where it is placed
window.geometry('%dx%d+%d+%d' % (w, h, x, y))
window.title("Led Calculator Assistant")

window.mainloop()
