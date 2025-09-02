import numpy as np
from math import cos, sin, acos, atan2, sqrt, pi
import time
import serial

# Replace with your Arduino's serial port (e.g., COM3 for Windows, /dev/ttyUSB0 or /dev/ttyACM0 for Linux/Mac)
ROBOT_SERIAL_PORT = "/dev/tty.usbmodem11201"

ROBOT_BAUD_RATE = 9600


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

def send_command2arm(ser, command):
  """Send a command to arm and wait for an 'Ok' response."""
  ser.write((command + "\n").encode())  # Send command with newline
  print(f"Sent: {command.strip()}")

# ------------------------------
# SIMULATION PARAMETERS
# ------------------------------
L1 = 25.3       # Length of the first link
L2 = 21.0       # Length of the second link

# Arm base positions (left and right)
base_left = np.array([-23.9, 0])
base_right = np.array([23.9, 0])

# Initial configuration: both arms pointing upward.
# For the left arm we choose the "elbow-up" solution (theta2 = -acos(...)),
# while for the right (blue) arm we later select the alternative branch.
current_angles = {
    'left': (pi/2, 0),
    'right': (pi/2, 0)
}


# ------------------------------
# FUNCTION TO ANIMATE ARM MOVES
# ------------------------------
def position_arms(x1, y1, x2, y2):
    """
    Animates moving the arms to new target positions.
      - The left arm (with base_left) moves to (x1, y1).
      - The right arm (with base_right) moves to (x2, y2).
    
    The right arm uses the positive_solution branch so its elbow points right.
    
    Parameters:
      x1, y1: Target for left arm.
      x2, y2: Target for right arm.
      steps: Number of interpolation steps.
      pause_time: Pause duration between steps.
      left_end_color: Color for left arm's end effector.
      right_end_color: Color for right arm's end effector.
    """
    global current_angles, ax
    
    command = "#LAC "+ str(23.9+x1) + "," + str(y1)
    #print(command)
    if command:
      send_command(ser, command)  # Send command and wait for Ok
    command = "#RAC " + str(x2-23.9) + "," + str(y2)
    #print(command)
    if command:
      send_command(ser, command)  # Send command and wait for Ok

# ------------------------------
# NEW GRID SIMULATION: CUSTOM p1/p2 ASSIGNMENT
# ------------------------------
def simulate_grid_custom(ser):
    """
    For this simulation:
      - p1 iterates over all grid points in the workspace in snake order.
      - For each fixed p1, p2 iterates over all grid points in snake order.
      (The rest of the function remains unchanged.)
    """
    # Create a 10x10 grid inside the workspace rectangle.
    grid_x = np.linspace(-9.6, 9.6, 10)
    grid_y = np.linspace(5, 25, 10)
    grid_points = [
        (x, y)
        for i, x in enumerate(grid_x)
        for y in (grid_y if i % 2 == 0 else grid_y[::-1])
    ]
    
    # For each new p1, we initialize the arm holding p1 to the left arm.
    for p1 in grid_points:
        p1_arm = 'left'  # the arm that holds p1 (shows yellow end)
        # For each fixed p1, let p2 iterate over all grid points.
        for p2 in grid_points:
            # Since p1 and p2 follow the same snake route, decide based on their ordering.
            # Use left arm for all points that come before the meeting point (p2's index < p1's index)
            # and right arm for the meeting point and those after (p2's index >= p1's index).
            idx_p1 = grid_points.index(p1)
            idx_p2 = grid_points.index(p2)
            if idx_p2 < idx_p1:
                candidate_arm = 'left'
            else:
                candidate_arm = 'right'
        
            # If the candidate arm is the one already holding p1, we must swap.
            if candidate_arm == p1_arm:
                p1_arm = 'right' if p1_arm == 'left' else 'left'
            
            # The free arm (not holding p1) is used for p2.
            free_arm = 'right' if p1_arm == 'left' else 'left'
            
            # Now call position_arms accordingly.
            if idx_p1 != idx_p2:
                if p1_arm == 'left':
                    position_arms(p1[0], p1[1], p2[0], p2[1])
                else:
                    position_arms(p2[0], p2[1], p1[0], p1[1])
                time.sleep(1)
            
    print("Custom grid simulation complete.")

# ------------------------------
# MAIN: RUN THE CUSTOM GRID SIMULATION
# ------------------------------
if __name__ == '__main__':
  try:
    ser = serial.Serial(ROBOT_SERIAL_PORT, ROBOT_BAUD_RATE, timeout=1)


    time.sleep(2)  # Wait for Arduino to reset

    simulate_grid_custom(ser)

    ser.close()

  except serial.SerialException as e:
      print(f"Error: {e}")
  except FileNotFoundError:
      print("Error: Command file not found.")
