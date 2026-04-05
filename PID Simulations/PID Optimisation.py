import numpy as np
import matplotlib.pyplot as plt

plt.style.use('seaborn-v0_8-whitegrid') 

# Simulation domain and system properties
start = 0
end = 3
dt = 0.071
# dt = 0.001 # Appriximately continuous time
thrust = 74
moment_arm = 0.28
I = 0.229
time_frame = np.arange(start, end + dt, dt)

# PD controller
class PDController():
    def __init__(self, Kp, Kd): # Constructor initialise gains
        self.Kp = Kp
        self.Kd = Kd

    def compute(self, yaw, yaw_rate, yaw_ref=0):
        error = yaw_ref - yaw # Error
        return self.Kp * error - self.Kd * yaw_rate # Return PD output

# Simulate discrete time response to step input
def sim(controller, yaw, yaw_rate, time_frame):
    yaw_history = []
    tvc_history = []
    yaw = np.radians(yaw)
    yaw_rate = np.radians(yaw_rate)

    for _ in time_frame: # Walk forward through time steps and compute dynamics
        yaw_history.append(yaw)
        tvc = controller.compute(yaw, yaw_rate)
        tvc = np.clip(tvc, -26*np.pi/180, 26*np.pi/180) # TVC limit (26 degrees converted to radians)
        tvc_history.append(tvc)

        torque = thrust * moment_arm * np.sin(tvc)

        alpha = torque / I

        yaw_rate = yaw_rate + alpha * dt
        yaw = yaw + yaw_rate * dt 
    return yaw_history, tvc_history

# Continuous time sweep
# Kp_set = np.arange(0.1, 1.0, 0.1)
# Kp_set_fixed = 0.9
# Kd_set = np.arange(0.01, 0.1, 0.01)
# Kd_set_fixed = 0.09

# Discrete Time - sweep through various values
Kp_set = np.arange(0.1, 0.25, 0.0125) * 5
Kp_set_fixed = 0.25 * 5
Kd_set = np.arange(0.01, 0.03, 0.002) * 5
Kd_set_fixed = 0.03 * 5

# Sweep various combinations
values = []
for Kd in Kd_set:
    values.append((Kp_set_fixed, Kd))
for Kp in Kp_set:
    values.append((Kp, Kd_set_fixed))

print("Gain Values (Kp, Kd): ", values)

iterations = len(values)
# Plot various iterations
fig, ax = plt.subplots(figsize=(10, 6)) 
cmap = plt.cm.turbo
norm = plt.Normalize(0, iterations - 1) 
for i, (Kp, Kd) in enumerate(values):
    controller = PDController(Kp=Kp, Kd=Kd)
    yaw_history, tvc_history = sim(controller, -10, 0, time_frame)
    
    color = cmap(norm(i))
    ax.plot(time_frame, np.degrees(yaw_history),
            color=color)

sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
cbar = fig.colorbar(sm, ax=ax)
cbar.set_label(r"Iteration Number")
cbar.ax.yaxis.set_major_formatter('{:.0f}'.format)

ax.set_xlabel(r"Time (s)")
ax.set_ylabel(r"Rocket Angle (deg)")
ax.set_title(r"Closed-Loop System Response to a $10^\circ$Correction (Step Input)")
ax.grid(alpha=0.3)
plt.tight_layout()
plt.show()

# controller = PDController(Kp=0.9, Kd=0.09) # Continuous time values

controller = PDController(Kp=1.25, Kd=0.15) # Discrete time optimised gain values

yaw_history, tvc_history = sim(controller, -10, 0, time_frame) # Step input of -10 with optimised values

# Plot of step input response
motor_y = np.linspace(min(yaw_history), max(tvc_history), 100) * 180 / np.pi
motor_t = np.zeros(100) + 1.2
motor_t2 = np.zeros(100) + 2.5
plt.figure(figsize=(10, 6))
plt.plot(time_frame, np.degrees(yaw_history), label=r'Rocket Angle (deg)')
plt.plot(time_frame, np.degrees(tvc_history), label=r'TVC Mount Angle (deg)')
plt.plot(time_frame, np.zeros(len(time_frame)), color='k', linestyle='--',  label=r'Target (0°)')
plt.plot(motor_t, motor_y, color='red', linestyle='--', label=r'First and Second Motor Burn Times (s)')
plt.plot(motor_t2, motor_y, color='red', linestyle='--')
plt.xlabel(r"Time (s)")
plt.ylabel(r"Angle (deg)")
plt.title(r"Response to a 10° Correction")
plt.legend(loc='upper left')
plt.grid(alpha=0.3)
plt.tight_layout()
plt.show()