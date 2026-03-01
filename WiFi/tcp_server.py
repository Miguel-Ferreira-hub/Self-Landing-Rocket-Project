import socket # Computer networking library
import threading # Thread library, lets code run in parallel so data transmission does not block other functions

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

def receiver(): # This function continuously reads data
    while True: # Infinite loop
        data = conn.recv(1024) # Receive data from ESP (up to 1024 bytes)
        if not data: # Exit loop if no data received
            break
        print("[ESP] ", data.decode().strip()) # Print received string 

# Start thread that listens to ESP
threading.Thread(target=receiver, daemon=True).start() # Runs receiver in parallel, daemon=True closes thread when program exits

# Print statements
print("\nYou can now type commands:")
print("Example: Parachute\n")

# Main thread -> Lets commands be typed and sent to the ESP within the Arduino
while True:
    cmd = input("")  # Type command goes here
    conn.send((cmd + "\n").encode()) # Sends command
