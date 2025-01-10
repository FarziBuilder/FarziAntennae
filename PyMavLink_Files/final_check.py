import time
import math
from pymavlink import mavutil
import serial

# Ground station coordinates (degrees)
GS_LAT = 40.0
GS_LON = -105.0
GS_ALT = 1600.0  # meters above sea level

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


def main():
    # Connect to telemetry (MAVLink)
    connection = mavutil.mavlink_connection('COM24', baud=57600)
    connection.wait_heartbeat()
    print("Telemetry Module: Heartbeat received")

    # Connect to Arduino/ESP32
    ser = serial.Serial('COM20', 115200, timeout=1)
    print("Connected to Arduino via Serial")

    while True:
        msg = connection.recv_match(blocking=True)
        if msg and msg.get_type() == "GPS_RAW_INT":
            # Extract UAV lat/lon/alt
            # NOTE: lat/lon in microdegrees, alt in millimeters
            uav_lat = float(msg.lat) / 1e7
            uav_lon = float(msg.lon) / 1e7
            uav_alt = float(msg.alt) / 1000.0

            # Compute az/el (spherical Earth approach)
            az_deg, el_deg = point_antenna_simple(
                GS_LAT, GS_LON, GS_ALT,
                uav_lat, uav_lon, uav_alt
            )

            # Build JSON-like string
            data = f"{{'az': {az_deg:.5f}, 'el': {el_deg:.5f}}}\n"
            ser.write(data.encode('utf-8'))
            print(f"Sent to Arduino: {data.strip()}")

        time.sleep(1)

if __name__ == "__main__":
    main()
