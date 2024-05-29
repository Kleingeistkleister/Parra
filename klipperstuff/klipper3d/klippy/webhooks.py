import serial
import time



def main():
    serial_port = '/dev/ttyS0'  # This is the device path found in dmesg
    baud_rate = 115200 
    
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud rate.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        while True:
            command = "get_config\n"  # Add newline if required by the device
            ser.write(command.encode())
            print(f"Sent: {command.strip()}")
            
            response = ser.readline().decode('utf-8').strip()
            print(f"Received: {response}")
            
            time.sleep(2)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
