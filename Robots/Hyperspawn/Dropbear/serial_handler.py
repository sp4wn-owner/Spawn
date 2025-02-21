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

def handle_serial(ser, message):
    if ser:
        try:
            # Assuming message is already a JSON string
            ser.write(message.encode() + b'\n')
            print('Data sent to serial:', message)
            # Here we send a response back to the process that called this script:
            sys.stdout.write("{\"status\": \"success\", \"message\": \"Data sent\"}\n")
            sys.stdout.flush()  # Ensure the output is immediately written
        except Exception as e:
            print('Error sending data to serial:', e)
            # Send an error response back:
            sys.stdout.write(f"{{\"status\": \"error\", \"message\": \"{e}\"}}\n")
            sys.stdout.flush()
    else:
        # If serial connection wasn't established, inform the parent process:
        sys.stdout.write("{\"status\": \"error\", \"message\": \"Serial connection not established\"}\n")
        sys.stdout.flush()

def main():
    ser = serial_connect('/dev/ttyUSB0', 115200)

    if ser:
        try:
            if len(sys.argv) > 1:
                handle_serial(ser, sys.argv[1])
            else:
                # Inform parent process if no message was provided:
                sys.stdout.write("{\"status\": \"error\", \"message\": \"No message provided\"}\n")
                sys.stdout.flush()
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        finally:
            ser.close()
            print("Serial connection closed.")
    else:
        print("Failed to initialize serial connection.")

if __name__ == "__main__":
    main()