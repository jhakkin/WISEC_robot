import serial
import struct


def read_serial_data(port: str, baudrate: int):
    try:
        # Open the serial port
        with serial.Serial(port, baudrate, timeout=1) as ser:
            #print(f"Listening on {port} at {baudrate} baud rate...")

            buffer = b''

            while True:
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

                    # Print the result
                    if(result < 5000): 
                        print(f"{result}")

    except serial.SerialException as e:
        print(f"Error opening the serial port: {e}")


if __name__ == "__main__":
    # Example usage
    read_serial_data(port="/dev/tty.usbmodem11402", baudrate=115200)
