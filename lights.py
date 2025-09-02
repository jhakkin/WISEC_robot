import time
import serial

# Replace with your Arduino's serial port (e.g., COM3 for Windows, /dev/ttyUSB0 or /dev/ttyACM0 for Linux/Mac)

RARM_SERIAL_PORT = "/dev/tty.usbmodem11402"
LARM_SERIAL_PORT = "/dev/tty.usbmodem11102"

ARM_BAUD_RATE = 115200

def initialize():
    global rarm_ser, larm_ser
    """Initialize serial connections to the right and left arms."""
    rarm_ser = serial.Serial(RARM_SERIAL_PORT, ARM_BAUD_RATE, timeout=1)
    #larm_ser = serial.Serial(LARM_SERIAL_PORT, ARM_BAUD_RATE, timeout=1)

def deinitialize():
    """Close the serial connections."""
    if rarm_ser.is_open:
        rarm_ser.close()
   # if larm_ser.is_open:
    #    larm_ser.close()

def send_command2arm(ser, command):
  """Send a command to arm and wait for an 'Ok' response."""
  ser.write(command.encode())  # Send command 
  print(f"Sent: {command.strip()}")

def rlight_on():
    """Turn on the right arm light."""
    send_command2arm(rarm_ser, "1")

def rlight_off():
    """Turn off the right arm light."""
    send_command2arm(rarm_ser, "0")

def llight_on():
    """Turn on the left arm light."""
    send_command2arm(larm_ser, "1")

def llight_off():
    """Turn off the left arm light."""
    send_command2arm(larm_ser, "0")

def rarm_read():
    """Read data from the right arm."""
    if rarm_ser.is_open:
        buffer = b''

        while True:
            data = rarm_ser.read(2)  # Read up to 1024 bytes

            if data:
                buffer += data

                # Process complete 2-byte chunks
                while len(buffer) >= 2:
                    # Extract two bytes (MSB first)
                    msb, lsb = buffer[:2]
                    buffer = buffer[2:]

                    # Convert bytes to 16-bit integer
                    result = (msb << 8) | lsb
                    return result if result < 5000 else None
    else:
        print("Right arm serial port is not open.")
        return None

def larm_read():
    """Read data from the left arm."""
    if larm_ser.is_open:
        buffer = b''

        while True:
            data = larm_ser.read(2)  # Read up to 1024 bytes

            if data:
                buffer += data

                # Process complete 2-byte chunks
                while len(buffer) >= 2:
                    # Extract two bytes (MSB first)
                    msb, lsb = buffer[:2]
                    buffer = buffer[2:]

                    # Convert bytes to 16-bit integer
                    result = (msb << 8) | lsb
                    return result if result < 5000 else None
    else:
        print("Left arm serial port is not open.")
        return None

# ------------------------------
# MAIN: RUN THE CUSTOM GRID SIMULATION
# ------------------------------
if __name__ == '__main__':
  try:
    initialize()

    time.sleep(2)  # Wait for Arduino to reset
    send_command2arm(rarm_ser, "1")
    #send_command2arm(larm_ser, "1")
    print(f"{rarm_read()}")  # Example read from right arm
    #print(f"{larm_read()}")  # Example read from left arm
    deinitialize()

  except serial.SerialException as e:
      print(f"Error: {e}")
  except FileNotFoundError:
      print("Error: Command file not found.")
