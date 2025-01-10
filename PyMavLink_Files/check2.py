from pymavlink import mavutil

# Connect to the MiniPix via the appropriate port
# Replace 'COMx' with your MiniPix's COM port on Windows or '/dev/ttyUSBx' on Linux
connection = mavutil.mavlink_connection('COM19', baud=57600)

# Wait for a heartbeat to ensure connection
connection.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Function to get roll, pitch, and yaw from ATTITUDE message
def get_attitude():
    while True:
        # Wait for an ATTITUDE message
        msg = connection.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            roll = msg.roll  # Roll in radians
            pitch = msg.pitch  # Pitch in radians
            yaw = msg.yaw  # Yaw in radians

            # Convert radians to degrees for easier interpretation (optional)
            roll_deg = roll * (180 / 3.14159)
            pitch_deg = pitch * (180 / 3.14159)
            yaw_deg = yaw * (180 / 3.14159)

            print(f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
            

# Call the function to continuously print roll, pitch, and yaw
get_attitude()
