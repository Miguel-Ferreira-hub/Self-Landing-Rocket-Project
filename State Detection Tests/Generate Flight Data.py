# This script aims to simulate fabricated sensor data for a rocket during flight
# altitude, velocity and acceleration are simulated 
# these can be input into the on-board rocket software to 
# testing guidance, navigation and control algorithms
import matplotlib.pyplot as plt # Imports
import numpy as np
import pandas as pd
import os

# OS path and code decisions
directory = r'C:\Users\migue\Desktop\Part IV\GDP\Generated Data'
path = os.path.join(directory, 'DATA.csv')
plt.style.use('ggplot')
noise_enabled = True # When set to True, sensor readings are simulated with noise

# Initialising arrays for simulated data
time = np.linspace(0,20,1000)
altitude = np.zeros(len(time))
velocity = np.zeros(len(time))
acceleration = np.zeros(len(time))

# Generating noise
noise_alt = np.random.normal(0,0.27,1000)
noise_vel = np.random.normal(0,1.2,1000)
noise_accel = np.random.normal(0,0.62,1000)

# Generating curves
for i in range(len(time)):
    if time[i] < 5 or time[i] > 15:
        altitude[i] = 0
        velocity[i] = 0
        acceleration[i] = -9.81
    else:
        altitude[i] = -2*(time[i]-10)**2 + 50
        velocity[i] = -4*(time[i] - 10)

    
for i in range(len(time)):
    if time[i] > 5 and time[i] < 7.18:
        acceleration[i] = -25*(time[i]-6)**2 + 25
    elif time[i] > 7.18:
        acceleration[i] = -9.81

# Adding noise
if noise_enabled == True:
    altitude += noise_alt
    velocity += noise_vel
    acceleration += noise_accel

# Plotting curves
plt.figure(figsize=(12,12))
plt.plot(time, altitude, color='red', label='Altitude (m)')
plt.plot(time, velocity, color='yellow', label='Velocity (ms-1)')
plt.plot(time, acceleration, color='green', label='Acceleration (ms-2)')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m), Velocity (ms-1), Acceleration (ms-2)')
plt.legend()
plt.title('Fabricated Data for State Detection Test')
plt.show()

# Producing data files
data = pd.DataFrame({'Time (s)': time, 'Acceleration (ms-2)': acceleration, 'Velocity (ms-1)': velocity, 'Altitude (m)': altitude})
data.to_csv(path, index=False)