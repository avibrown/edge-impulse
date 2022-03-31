import pandas as pd # Overpowered - but readable :)
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
df = pd.DataFrame(columns=['timestamp', 'current'])

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

'''
This loop runs for the duration of the timeout we set.
Every 10 milliseconds new data is polled from the ODrive
object and appended to the pandas dataframe.
'''
while timer < timeout:
    if  (last_time != timer):
        last_time = timer
        print(last_time)
        df = df.append({'timestamp': timer,
                        'current': odrv0.ibus},
                        ignore_index=True)

    # Set the name of this file as you see fit.
    df.to_csv('no_disturbance_15.csv', index=False)

vel_thread.join()
tim_thread.join()

print('Done.')
