import socket # Computer networking library
import threading # Thread library, lets code run in parallel so data transmission does not block other functions
import matplotlib.pyplot as plt # Plots
from matplotlib.animation import FuncAnimation # Real time plots

HOST = '0.0.0.0' # Listen on all network interfaces (WiFi, Ethernet, etc.)
PORT = 5001 # Listening port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Creates a tcp socket, (AF_INET -> IPv4, SOCK_STREAM -> TCP protocol)
s.bind((HOST, PORT)) # Attaches socket to IP and port
s.listen(1) # Listening mode

# Print Statements
print("GDP 23 - Self Landing Rocket")
print("Listening for ESP Transmission...")
conn, addr = s.accept() # Accept ESP connection
print("Connected to:", addr)

# Data storage
buffer = []
time_data = []
altitude_data = []

def receiver():
    while True:
        try:
            data = conn.recv(1024)
            if not data:
                break

            lines = data.decode().strip().split('\n')

            for line in lines:
                if line.startswith("DATA,"):
                    buffer.append(line)   # goes to plot only

                elif line.startswith("MSG,"):
                    print("[ESP] " + line.replace("MSG,", ""))  # prints only status messages

        except:
            break

# Start thread that listens to ESP
threading.Thread(target=receiver, daemon=True).start() # Runs receiver in parallel, daemon=True closes thread when program exits

# Print statements
print("\nYou can now type commands:")
print("Example: Parachute\n")

# Figure
plt.style.use('ggplot')
fig, ax = plt.subplots()

def update(frame): # Animation function (frame is the default argument for animation)
   while buffer:
    line = buffer.pop(0)

    parts = line.replace("DATA,", "").split(',')
    if len(parts) == 2:
        try:
            t = float(parts[0])
            alt = float(parts[1])
            time_data.append(t)
            altitude_data.append(alt)
        except:
            pass

    ax.clear() # Clears graph
    ax.plot(time_data, altitude_data) # Plot data
    ax.set_xlabel("Time") # Graph formating
    ax.set_ylabel("Altitude")
    ax.set_title("Live Telemetry")

animation = FuncAnimation(fig, update, interval=300) # Animation, updates every 100ms
plt.show(block=False)

# Main thread -> Lets commands be typed and sent to the ESP within the Arduino
while True:
    cmd = input("")  # Type command goes here
    conn.send((cmd + "\n").encode()) # Sends command