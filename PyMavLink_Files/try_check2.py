import serial
import time

def main():
    # -----------------------------
    # Configuration Parameters
    # -----------------------------
    
    SERIAL_PORT = 'COM20'      # Replace with your Arduino's serial port (e.g., 'COM3' or '/dev/ttyUSB0')
    BAUD_RATE = 115200        # Must match the baud rate in your Arduino code
    SEND_INTERVAL = 1         # Time interval between sends in seconds (adjust as needed)
    
    # Azimuth Angle Values
    AZIMUTH_START = 30.00      # Starting azimuth angle in degrees
    AZIMUTH_END = 60.00        # Ending azimuth angle in degrees
    AZIMUTH_STEP = 1.00        # Increment step for azimuth angle in degrees
    
    # Elevation Angle Values
    ELEVATION_START = 0.00      # Starting elevation angle in degrees
    ELEVATION_END = 45.00       # Ending elevation angle in degrees
    ELEVATION_STEP = 2.50       # Increment step for elevation angle in degrees
    
    # -----------------------------python 
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
        # Initialize both angles
        azimuth_angle = AZIMUTH_START
        elevation_angle = ELEVATION_START
        
        # Determine the maximum number of steps needed
        max_steps_azimuth = int((AZIMUTH_END - AZIMUTH_START) / AZIMUTH_STEP) + 1
        max_steps_elevation = int((ELEVATION_END - ELEVATION_START) / ELEVATION_STEP) + 1
        max_steps = max(max_steps_azimuth, max_steps_elevation)
        
        for step in range(max_steps):
            # Update Azimuth Angle
            if azimuth_angle <= AZIMUTH_END:
                current_azimuth = azimuth_angle
                azimuth_angle += AZIMUTH_STEP
            else:
                current_azimuth = AZIMUTH_END  # Keep at max if exceeded
            
            # Update Elevation Angle
            if elevation_angle <= ELEVATION_END:
                current_elevation = elevation_angle
                elevation_angle += ELEVATION_STEP
            else:
                current_elevation = ELEVATION_END  # Keep at max if exceeded
            
            # Prepare the command string with current azimuth and elevation angles
            command_str = f"{current_azimuth:.2f} {current_elevation:.2f}\n"
            
            # Send the command
            ser.write(command_str.encode('utf-8'))
            print(f"Sent: Azimuth = {current_azimuth}°, Elevation = {current_elevation}°")
            
            # Optional: Read response from Arduino
            time.sleep(0.5)  # Wait for Arduino to process
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').rstrip()
                if response:
                    print(f"Arduino: {response}")
            
            # Wait before sending the next set of angles
            time.sleep(SEND_INTERVAL)
        
        print(f"Azimuth angle reached {AZIMUTH_END}° and Elevation angle reached {ELEVATION_END}°. Stopping the program.")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    
    finally:
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
