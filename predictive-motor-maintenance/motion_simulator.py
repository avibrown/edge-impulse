import odrive       # Need to have odrivetool installed
import time         # To set sampling frequency
import threading    # For running a backround timer thread
import math         # For cosine function
import random

'''
Select how long you'd like to collect data for.
This will run for 60 sec.
'''
timeout = 120 * 1000

timer = 0           # Initialize timer variable
start = False
last_time = -1           # To avoid repeated entries

'''
Create a pandas dataframe to store our
precious data! This will be exported as a .CSV file
at the end of the script.
'''

odrv0 = odrive.find_any() # Create ODrive object

'''
Runs in the background to increment our timer
in jumps of 10.
'''
def increment_timer():
    global timer
    while (timer < timeout):
        time.sleep(0.01)
        timer += 10

'''
This function will also run in a background thread
and will be responsible for changing the velocity
according to a vertically-transformed cosine.
'''
def vel_change():
    global timer
    while (timer < timeout):
        c = random.randint(5, 10)
        y = 0
        while y < (2 * math.pi):
            odrv0.axis0.controller.input_vel = (2 * math.cos(c * y)) + 10
            y += 0.8
            time.sleep(0.1)
        
    odrv0.axis0.controller.input_vel = 5 # Stop motor when done


vel_thread = threading.Thread(target=vel_change, args = ())
tim_thread = threading.Thread(target=increment_timer, args = ())

vel_thread.daemon = True
tim_thread.daemon = True

vel_thread.start()  # Start your engines...

_ = input('Hit enter to start data collection...')

tim_thread.start()  # Start timer thread

vel_thread.join()
tim_thread.join()

print('Done.')
