import serial
import struct
from collections import deque
import time

RARM_SERIAL_PORT = "/dev/tty.usbmodem11402"
LARM_SERIAL_PORT = "/dev/tty.usbmodem11102"

def read_serial_data(port: str, baudrate: int):
    try:
        # Open the serial port
        with serial.Serial(port, baudrate, timeout=1) as ser:
            #print(f"Listening on {port} at {baudrate} baud rate...")

            buffer = b''
            window = deque(maxlen=100)  # Stores the last 1000 values
            running_sum = 0  # Maintains the sum for efficient average computation
            running_avg = 0
            running_min = 0
            running_max = 0

            for i in range(101):
                # Read data from the serial port
                data = ser.read(10)  # Read up to 1024 bytes

                if data:
                    #print(f"Received data: {data}")
                    pos = data.find(b'$$$')
                    #print(f"Position of '$$$': {pos}")
                    # Extract two bytes (MSB first)
                    msb, lsb = data[pos+3:pos+5]

                    # Convert bytes to 16-bit integer
                    result = (msb << 8) | lsb

                    # Only use values less than 5000
                    if result < 5000:
                        # Update running sum and window
                        if len(window) == 100:
                            running_sum -= window[0]  # remove the oldest value from sum
                        window.append(result)
                        running_sum += result

                        # Calculate statistics
                        running_avg = running_sum / len(window)
                        running_min = min(window)
                        running_max = max(window)

            # Print results
            #print(
            #    f"{result},"
            #    f"{running_avg:.2f},"
            #    f"{running_min},{running_max},{running_max-running_min} "
            #)
            return running_max-running_min

    except serial.SerialException as e:
        print(f"Error opening the serial port: {e}")


if __name__ == "__main__":
    # Example usage
    while True:
        print(read_serial_data(port=LARM_SERIAL_PORT, baudrate=115200))