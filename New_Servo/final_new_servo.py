from pymavlink import mavutil
import serial
import math
import time

# Replace with actual COM ports and baud rates
mav_connection = mavutil.mavlink_connection('COM18', baud=57600)
ser = serial.Serial('COM20', 115200, timeout=1)

# Wait for a heartbeat from the MAVLink device
mav_connection.wait_heartbeat()
print("Heartbeat received")

# Ground Station Coordinates
GS_LAT = 26.776143671572395
GS_LON = 80.99542609984692
GS_ALT = 125.0  # meters above sea level

# Orientation parameters
orient_angle = 0.0
mid_value = 90.0

uav_lat, uav_lon, uav_alt = 0 , 0, 0

def point_antenna_simple(gs_lat1, gs_lon1, gs_alt1, uav_lat1, uav_lon1, uav_alt1):
    """
    Computes azimuth and elevation using a spherical Earth model.
    Azimuth = 0° is north, 90° is east.
    """
    R = 6371000.0  # Earth radius in meters

    # Convert degrees to radians
    gs_lat_r = math.radians(gs_lat1)
    gs_lon_r = math.radians(gs_lon1)
    uav_lat_r = math.radians(uav_lat1)
    uav_lon_r = math.radians(uav_lon1)

    # Calculate deltas
    delta_phi = uav_lat_r - gs_lat_r
    delta_lambda = uav_lon_r - gs_lon_r
    phi_avg = 0.5 * (gs_lat_r + uav_lat_r)

    # Convert lat/lon deltas to x, y in meters (local tangent plane)
    x = R * delta_lambda * math.cos(phi_avg)  # East-West
    y = R * delta_phi                         # North-South

    # Ground distance
    ground_dist = math.sqrt(x**2 + y**2)

    # Azimuth
    az_radians = math.atan2(x, y)
    az_deg = math.degrees(az_radians) % 360.0
    az_deg = az_deg - 360 if az_deg > 180 else az_deg
    
    print(f"ground dist: {ground_dist}")
    print(f"az_radians: {az_radians}")
    print(f"az_deg: {az_deg}")

    # Elevation
    alt_diff = uav_alt1 - gs_alt1
    el_radians = math.atan2(alt_diff, ground_dist)
    el_deg = math.degrees(el_radians)
    print(f"alt_diff: {alt_diff}")
    print(f"el_radians: {el_radians}")
    print(f"el_deg: {el_deg}")
   
    return az_deg, el_deg
 
def process_gps_raw_int(msg):   
    global uav_lat, uav_lon, uav_alt
    if msg.get_type() == "GPS_RAW_INT":
        # Extract latitude, longitude, and altitude
        uav_lat = msg.lat / 1e7  # Convert microdegrees to degrees
        uav_lon = msg.lon / 1e7  # Convert microdegrees to degrees
        uav_alt = 125 #msg.alt / 1000  # Convert millimeters to meters
        print(f"Latitude: {uav_lat}°, Longitude: {uav_lon}°, Altitude: {uav_alt} meters")

# Read and process incoming messages
while True:
    msg = mav_connection.recv_match(blocking=True)
    if msg.get_type() == "GPS_RAW_INT": 
        process_gps_raw_int(msg)
        az_deg, el_deg = point_antenna_simple(
            GS_LAT, GS_LON, GS_ALT,
            uav_lat, uav_lon, uav_alt  # Use actual UAV altitude
        )
        
        final_az_deg = az_deg - orient_angle
        if final_az_deg < -180:
            final_az_deg += 360
        
        final_az_deg = mid_value - final_az_deg
        
        final_el_deg = 30 + (40 - el_deg)
        if final_el_deg < 0:
            final_el_deg = 30

        # Create a message with start and end markers
        data = f"<{final_az_deg:.5f},{final_el_deg:.5f}>\n"
        ser.write(data.encode('utf-8'))
        print(f"Sent to Arduino: {data.strip()}")

        # Wait for Arduino to process
        time.sleep(0.5)

        # Read Arduino response
        while ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').rstrip()
            if response:
                print(f"Arduino: {response}")
        
        # Additional delay to prevent rapid sending
        time.sleep(1)
