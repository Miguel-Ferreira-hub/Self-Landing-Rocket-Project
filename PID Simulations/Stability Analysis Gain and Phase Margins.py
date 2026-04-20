import numpy as np
import matplotlib.pyplot as plt

plt.style.use('seaborn-v0_8-whitegrid')

I = 0.229 # Mass moment of inertia
dt = 0.071 # Sample time of the Arduino loop (discrete time step)

# Define dictionaries containing ascent and descent case dynamics
ascent_case = {
    "thrust": 74, # Average motor thrust 
    "moment_arm": 0.363, # Moment arm of TVC mount, thrust action point to centre of mass distance
    "Kp": 0.96, # Optimised proportional gain
    "Kd": 0.12, # Optimised derivative gain
    "actuator_lag": 0.025, # Servo motor lag (SG90 around 0.1s/60, maximum angle TVC mount can move is 15 deg)
    "name": "Ascent" # Name
}

descent_case = { # Descent case dynamics
    "thrust": 20,
    "moment_arm": 0.28,
    "Kp": 4.09,
    "Kd": 0.51,
    "actuator_lag": 0.025, 
    "name": "Descent"
}

f = np.logspace(-3,5,1000) # Frequency range

omega = f * 2 * np.pi

fig, ax = plt.subplots(2, 2, figsize=(14, 8)) 
fig.suptitle("Bode Plot", fontsize=14, fontweight="bold")
cases = [ascent_case, descent_case]

# Continuous time
for i, case in enumerate(cases):
    L = (1/(-omega**2*I))*(case["thrust"] * case["moment_arm"] * (case["Kp"] + omega*case["Kd"]*1j))*(1/((case["actuator_lag"] * omega * 1j) + 1)) # Open loop transfer function
    gain = 20 * np.log10(np.abs(L)) # Work out gain
    phase = np.unwrap(np.angle(L)) # Work out phase
    # Phase margin
    mask_pm = np.argmin(np.abs(gain))
    gain_crossover_frequency = omega[mask_pm]
    phase_margin = np.pi + phase[mask_pm]
    # Gain margin
    mask_gm = np.argmin(np.abs(phase + np.pi))
    phase_crossover_frequency= omega[mask_gm]
    gain_margin = -gain[mask_gm]
    print(f"--------------------{case['name']}--------------------")
    print(f"Gain Margin (dB): {gain_margin}")
    print(f"Phase Margin (rad): {phase_margin}")
    print(f"Phase Margin (deg): {np.rad2deg(phase_margin)}")

    ax[0,i].semilogx(omega, gain, label="Gain (dB)")
    ax[0,i].set_title(f"{case['name']}", fontsize=12, fontweight="bold")
    ax[0,i].set_ylabel("Gain (dB)", fontsize=12, fontweight="bold")
    ax[0,i].set_xlabel("Frequency (rad/s)", fontsize=12, fontweight="bold")
    ax[0,i].grid(alpha=0.7)
    ax[1,i].semilogx(omega, phase, label="Phase (rad)")
    ax[1,i].set_ylabel("Phase (rad)", fontsize=12, fontweight="bold")
    ax[1,i].set_xlabel("Frequency (rad/s)", fontsize=12, fontweight="bold")
    ax[1,i].grid(alpha=0.7)

plt.tight_layout()
plt.show()

fig, ax = plt.subplots(2, 2, figsize=(14, 8)) 
fig.suptitle("Discrete Time Bode Plot", fontsize=14, fontweight="bold")
cases = [ascent_case, descent_case]

# Discrete time
for i, case in enumerate(cases):
    omega = np.linspace(1e-3, np.pi / dt, 1000) # Up to Nyquist frequency
    z = np.exp(omega * dt * 1j)
    # Open loop transfer function
    s = (2/dt)*(1-z**-1)/(1+z**-1)
    controller = case["Kp"] + case["Kd"]*s
    simple_lag = 1/(case["actuator_lag"]*s + 1)
    plant = case["thrust"]*case["moment_arm"]/I * 1/s**2
    L = controller * simple_lag * plant
    gain = 20 * np.log10(np.abs(L)) # Work out gain
    phase = np.unwrap(np.angle(L)) # Work out phase
    # Phase margin
    mask_pm = np.argmin(np.abs(gain))
    gain_crossover_frequency = omega[mask_pm]
    phase_margin = np.pi + phase[mask_pm]
    # Gain margin
    mask_gm = np.argmin(np.abs(phase + np.pi))
    phase_crossover_frequency= omega[mask_gm]
    gain_margin = -gain[mask_gm]
    print(f"--------------------{case['name']}--------------------")
    print(f"Gain Margin (dB): {gain_margin}")
    print(f"Phase Margin (rad): {phase_margin}")
    print(f"Phase Margin (deg): {np.rad2deg(phase_margin)}")

    ax[0,i].semilogx(omega, gain, label="Gain (dB)")
    ax[0,i].set_title(f"{case['name']}", fontsize=12, fontweight="bold")
    ax[0,i].set_ylabel("Gain (dB)", fontsize=12, fontweight="bold")
    ax[0,i].set_xlabel("Frequency (rad/s)", fontsize=12, fontweight="bold")
    ax[0,i].grid(alpha=0.7)
    ax[1,i].semilogx(omega, phase, label="Phase (rad)")
    ax[1,i].set_ylabel("Phase (rad)", fontsize=12, fontweight="bold")
    ax[1,i].set_xlabel("Frequency (rad/s)", fontsize=12, fontweight="bold")
    ax[1,i].grid(alpha=0.7)

plt.tight_layout()
plt.show()

# Continuous and discrete time plotted together
fig, ax = plt.subplots(2, 2, figsize=(14, 8)) 
fig.suptitle("System Bode Plot", fontsize=14, fontweight="bold")
cases = [ascent_case, descent_case]

# Plot
for i, case in enumerate(cases):
    omega = np.linspace(1e-3, np.pi / dt, 1000) # Up to Nyquist frequency
    z = np.exp(omega * dt * 1j)
    # Open loop transfer function
    s = (2/dt)*(1-z**-1)/(1+z**-1)
    controller = case["Kp"] + case["Kd"]*s
    simple_lag = 1/(case["actuator_lag"]*s + 1)
    plant = case["thrust"]*case["moment_arm"]/I * 1/s**2
    L = controller * simple_lag * plant
    gain = 20 * np.log10(np.abs(L)) # Work out gain
    phase = np.unwrap(np.angle(L)) # Work out phase
    L_c = (1/(-omega**2*I))*(case["thrust"] * case["moment_arm"] * (case["Kp"] + omega*case["Kd"]*1j))*(1/((case["actuator_lag"] * omega * 1j) + 1)) # Open loop transfer function
    gain_c = 20 * np.log10(np.abs(L_c)) # Work out gain
    phase_c = np.unwrap(np.angle(L_c)) # Work out phase
    # Phase margin
    mask_pm = np.argmin(np.abs(gain))
    gain_crossover_frequency = omega[mask_pm]
    phase_margin = np.pi + phase[mask_pm]
    # Gain margin
    mask_gm = np.argmin(np.abs(phase + np.pi))
    phase_crossover_frequency= omega[mask_gm]
    gain_margin = -gain[mask_gm]
    print(f"--------------------{case['name']}--------------------")
    print(f"Gain Margin (dB): {gain_margin}")
    print(f"Phase Margin (rad): {phase_margin}")
    print(f"Phase Margin (deg): {np.rad2deg(phase_margin)}")

    ax[0,i].semilogx(omega, gain, label="Discrete")
    ax[0,i].semilogx(omega, gain_c, label="Continuous")
    ax[0,i].set_title(f"{case['name']}", fontsize=12, fontweight="bold")
    ax[0,i].set_ylabel("Gain (dB)", fontsize=12, fontweight="bold")
    ax[0,i].set_xlabel("Frequency (rad/s)", fontsize=12, fontweight="bold")
    ax[0,i].legend(fontsize=12)
    ax[0,i].grid(alpha=0.7)
    ax[1,i].semilogx(omega, phase, label="Discrete")
    ax[1,i].semilogx(omega, phase_c, label="Continuous")
    ax[1,i].set_ylabel("Phase (rad)", fontsize=12, fontweight="bold")
    ax[1,i].set_xlabel("Frequency (rad/s)", fontsize=12, fontweight="bold")
    ax[1,i].legend(fontsize=12)
    ax[1,i].grid(alpha=0.7)

plt.tight_layout()
plt.show()