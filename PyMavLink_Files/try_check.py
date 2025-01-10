import serial
import time

def main():
    # -----------------------------
    # Configuration Parameters
    # -----------------------------
    
    SERIAL_PORT = 'COM20'      # Replace with your Arduino's serial port (e.g., 'COM3' or '/dev/ttyUSB0')
    BAUD_RATE = 115200        # Must match the baud rate in your Arduino code
    SEND_INTERVAL = 1         # Time interval between sends in seconds (adjust as needed)
    
    # Fixed Angle Values
    AZIMUTH_ANGLE = 45.00     # Azimuth angle in degrees
    ELEVATION_START = 0.00    # Starting elevation angle in degrees
    ELEVATION_END = 45.00     # Ending elevation angle in degrees
    ELEVATION_STEP = 1      # Increment step for elevation angle in degrees
    
    # -----------------------------
    # Initialize Serial Connection
    # -----------------------------
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
        time.sleep(2)  # Wait for Arduino to reset
        ser.flushInput()
        print("Connected to Arduino.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}.")
        print(e)
        return
    
    try:
        elevation_angle = ELEVATION_START
        while elevation_angle <= ELEVATION_END:
            # Prepare the command string with current elevation angle
            command_str = f"{AZIMUTH_ANGLE:.2f} {elevation_angle:.2f}\n"
            
            # Send the command
            ser.write(command_str.encode('utf-8'))
            print(f"Sent: Azimuth = {AZIMUTH_ANGLE}°, Elevation = {elevation_angle:.2f}°")
            
            # Optional: Read response from Arduino
            time.sleep(0.5)  # Wait for Arduino to process
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').rstrip()
                if response:
                    print(f"Arduino: {response}")
            
            # Increment the elevation angle
            elevation_angle += ELEVATION_STEP
            
            # Wait before sending the next set of angles
            time.sleep(SEND_INTERVAL)
        
        print(f"Elevation angle reached {ELEVATION_END}°. Stopping the program.")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    
    finally:
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
