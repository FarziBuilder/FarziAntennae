import serial
import time

def main():
    # Make sure the 'COM3' and '9600' match your Arduino's port and baud rate
    ser = serial.Serial('COM20', 9600, timeout=1)
    time.sleep(2)  # Give the Arduino time to reset after opening the serial port

    value = 42  # Any integer or string you want to send

    # Send the value
    ser.write(str(value).encode('utf-8'))
    print(f"Sent to Arduino: {value}")

    # Optionally wait to read any response from Arduino
    time.sleep(0.5)
    while ser.in_waiting > 0:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"Arduino says: {response}")

    ser.close()

if __name__ == "__main__":
    main()
