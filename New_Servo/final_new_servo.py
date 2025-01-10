from pymavlink import mavutil
import serial
import math
import time

# Replace 'COM21' with the actual COM port of your telemetry module
connection = mavutil.mavlink_connection('COM18', baud=57600)

# Wait for a heartbeat from any system
connection.wait_heartbeat()
print("Heartbeat received")

uav_lat, uav_lon, uav_alt = 0, 0, 0
GS_LAT = 40.0
GS_LON = -105.0
GS_ALT = 1600.0  # meters above sea level

orient_angle = 60.0;
mid_value = 60.0;

ser = serial.Serial('COM20', 115200, timeout=1)


def point_antenna_simple(gs_lat, gs_lon, gs_alt, uav_lat, uav_lon, uav_alt):
    """
    Computes azimuth/elevation using a spherical Earth model.
    Azimuth = 0° is north, 90° is east.
    """

    # Earth radius (meters)
    R = 6371000.0  

    # Convert degrees -> radians directly
    gs_lat_r = gs_lat * math.pi / 180.0
    gs_lon_r = gs_lon * math.pi / 180.0
    uav_lat_r = uav_lat * math.pi / 180.0
    uav_lon_r = uav_lon * math.pi / 180.0

    # Calculate deltas
    delta_phi = uav_lat_r - gs_lat_r
    delta_lambda = uav_lon_r - gs_lon_r
    phi_avg = 0.5 * (gs_lat_r + uav_lat_r)

    # Convert lat/lon deltas to x, y in meters (local tangent plane)
    x = R * delta_lambda * math.cos(phi_avg)  # East-West
    y = R * delta_phi                         # North-South

    # Ground distance
    ground_dist = math.sqrt(x*x + y*y)

    # Azimuth
    az_radians = math.atan2(x, y)
    az_deg = math.degrees(az_radians) % 360.0

    # Elevation
    alt_diff = uav_alt - gs_alt
    el_radians = math.atan2(alt_diff, ground_dist)
    el_deg = math.degrees(el_radians)

    return az_deg, el_deg

# Function to process GPS_RAW_INT messages
def process_gps_raw_int(msg):
    if msg.get_type() == "GPS_RAW_INT":
        # Extract latitude, longitude, and altitude
        uav_lat = msg.lat / 1e7  # Convert microdegrees to degrees
        uav_lon = msg.lon / 1e7  # Convert microdegrees to degrees
        uav_alt = msg.alt / 1000  # Convert millimeters to meters
        
        print(f"Latitude: {uav_lat}°, Longitude: {uav_lon}°, Altitude: {uav_alt} meters")

# Read and process incoming messages
while True:
    msg = connection.recv_match(blocking=True)
    print("hello")
    if msg:
        process_gps_raw_int(msg)
        az_deg, el_deg = point_antenna_simple(
                GS_LAT, GS_LON, GS_ALT,
                uav_lat, uav_lon, uav_alt
            )
        final_az_deg = az_deg - orient_angle;
        
        final_az_deg += mid_value;
        
        final_el_deg = 30 + (45 - el_deg);
        
        
        data = f"{final_az_deg:.5f} {final_el_deg:.5f}\n"
        ser.write(data.encode('utf-8'))
        print(f"Sent to Arduino: {data.strip()}")
        time.sleep(0.5)  # Wait for Arduino to process
        while ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').rstrip()
            if response:
                print(f"Arduino: {response}")
            
            # Wait before sending the next set of angles
        time.sleep(1)
        
        
