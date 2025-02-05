# pip install pyserial
import sys
import serial

def serial_connect(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to serial port {port}")
        return ser
    except serial.SerialException as e:
        print(f"Serial connection failed: {e}")
        return None

def handle_serial(message):
    ser = serial_connect('/dev/ttyUSB0', 115200)
    if ser:
        try:
            # Assuming message is already a JSON string
            ser.write(message.encode() + b'\n')  # Add newline for line-based protocols
            print('Data sent to serial:', message)
        except Exception as e:
            print('Error sending data to serial:', e)
        finally:
            ser.close()  # It's good practice to close the serial port when done
    else:
        print("Serial connection not established, data not sent.")

if __name__ == "__main__":
    # The message will be passed as a command-line argument
    if len(sys.argv) > 1:
        handle_serial(sys.argv[1])
    else:
        print("No message provided to send to serial.")