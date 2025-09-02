import serial
import time

# Replace with your Arduino's serial port (e.g., COM3 for Windows, /dev/ttyUSB0 or /dev/ttyACM0 for Linux/Mac)
SERIAL_PORT = "/dev/tty.usbmodem4013201"
BAUD_RATE = 9600
TEXT_FILE = "moves.txt"

def send_command(ser, command):
    """Send a command to Arduino and wait for an 'Ok' response."""
    ser.write((command + "\n").encode())  # Send command with newline
    print(f"Sent: {command.strip()}")
    
    while True:
        response = ser.readline().decode().strip()  # Read Arduino response
        if response == "Ok":
            print("Received: Ok")
            break
        else:
            print(f"Received: {response}")

def main():
    """Reads a text file and sends commands to Arduino via serial."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset

        with open(TEXT_FILE, "r") as file:
            for line in file:
                command = line.strip()
                if command:
                    send_command(ser, command)  # Send command and wait for Ok

        ser.close()
        print("All commands sent successfully.")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except FileNotFoundError:
        print("Error: Command file not found.")

if __name__ == "__main__":
    main()
