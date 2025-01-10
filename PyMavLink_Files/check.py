from pymavlink import mavutil

# Replace 'COM21' with the actual COM port of your telemetry module
connection = mavutil.mavlink_connection('COM18', baud=57600)

# Wait for a heartbeat from any system
connection.wait_heartbeat()
print("Heartbeat received")

# Function to process GPS_RAW_INT messages
def process_gps_raw_int(msg):
    if msg.get_type() == "GPS_RAW_INT":
        # Extract latitude, longitude, and altitude
        lat = msg.lat / 1e7  # Convert microdegrees to degrees
        lon = msg.lon / 1e7  # Convert microdegrees to degrees
        alt = msg.alt / 1000  # Convert millimeters to meters
        
        print(f"Latitude: {lat}°, Longitude: {lon}°, Altitude: {alt} meters")

# Read and process incoming messages
while True:
    msg = connection.recv_match(blocking=True)
    if msg:
        process_gps_raw_int(msg)
