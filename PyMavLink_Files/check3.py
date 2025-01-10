import time
from pymavlink import mavutil
import serial

def main():
    # Connect to telemetry module (replace 'COM21' and baud rate if needed)
    connection = mavutil.mavlink_connection('COM21', baud=57600)
    connection.wait_heartbeat()
    print("Telemetry Module: Heartbeat received")

    # Connect to ESP32 via Serial (replace 'COM5' and baud rate as needed)
    ser = serial.Serial('COM5', 115200, timeout=1)
    print("Connected to ESP32 via Serial")

    while True:
        # Wait for GPS message
        msg = connection.recv_match(blocking=True)
        if msg and msg.get_type() == "GPS_RAW_INT":
            # Extract lat, lon, alt
            lat = msg.lat / 1e7   # Convert microdegrees to degrees
            lon = msg.lon / 1e7   # Convert microdegrees to degrees
            alt = msg.alt / 1000  # Convert millimeters to meters

            # Create a JSON-like string
            data = f"{{'lat': {lat}, 'lon': {lon}, 'alt': {alt}}}\n"
            ser.write(data.encode('utf-8'))
            print(f"Sent to ESP32: {data.strip()}")

        time.sleep(1)  # Adjust rate as needed

if __name__ == "__main__":
    main()
