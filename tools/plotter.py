from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from serial import Serial
from time import time

port = "/dev/ttyACM0"
baudrate = 115200
serial_port = Serial(port=port, baudrate=baudrate, timeout=1)

pitch_str = "Pitch: "
roll_str = "Roll: "

t =[]
pitch = []
roll = []

# Setup plot
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

def update(i, t, pitch, roll):
    # Read in line from serial output
    line = serial_port.readline().decode("utf-8").rstrip()

    # Parse line for pitch and roll data
    pitch_str_idx = line.find(pitch_str)
    roll_str_idx = line.find(roll_str)
    if pitch_str_idx != -1 and roll_str_idx != -1:
        pitch_start_idx = pitch_str_idx + len(pitch_str)
        pitch_end_idx = pitch_start_idx + line[pitch_start_idx:].find(" ")
        try:
            pitch.append(float(line[pitch_start_idx:pitch_end_idx]))
        except ValueError:
            pass
        roll_start_idx = roll_str_idx + len(roll_str)
        roll_end_idx = roll_start_idx + line[roll_start_idx:].find(" ")
        try:
            roll.append(float(line[roll_start_idx:roll_end_idx]))
        except ValueError:
            pass

        # Record time
        t.append(time())

        # Limit pitch and roll to 50 data points
        x = t[-50:]
        y1 = pitch[-50:]
        y2 = roll[-50:]

        # Draw x and y lists
        ax.clear()
        ax.plot(x, y1, label="Pitch")
        ax.plot(x, y2, label="Roll")

        # Format plot
        plt.title("Pitch and Roll")
        plt.ylabel("Degrees")
        plt.legend()

# Set up plot to call update() function periodically
_ = FuncAnimation(fig, update, fargs=(t, pitch, roll), interval=100)
plt.show()
