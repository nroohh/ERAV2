import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import serial

# === SETTINGS ===
PORT = "/dev/tty.SLAB_USBtoUART"  # Update this if needed
BAUD_RATE = 115200
INTERVAL = 10  # milliseconds
HISTORY_SIZE = 25000
CSV_FILE = "magnetometer_output.csv"

# === Serial and Data Storage ===
serialport = None
raw_data = []  # List of [x, y, z] strings

# For plotting (converted to float)
mag_x = deque(maxlen=HISTORY_SIZE)
mag_y = deque(maxlen=HISTORY_SIZE)

# === Read from serial port ===
def get_imu_data():
    global serialport
    if not serialport:
        serialport = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        print("Opened", serialport.name)
        time.sleep(3)
        serialport.readline()  # Flush first line

    line = serialport.readline().decode('utf-8').strip()
    if not line:
        return None

    parts = line.split(',')
    if len(parts) != 3:
        return None

    raw_data.append(parts)  # Save as list of strings
    return parts

# === Plotting ===
fig, ax = plt.subplots()

def animate(i):
    for _ in range(5):  # Increase if you want faster updates
        ret = get_imu_data()
        if not ret:
            continue
        x_str, y_str, _ = ret

        try:
            x = float(x_str)
            y = float(y_str)
            mag_x.append(x)
            mag_y.append(y)
        except ValueError:
            continue

    ax.cla()
    ax.set_title("Live IMU Plot: X vs Y")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    # Automatically adjust to real range (no clipping to -1~1)
    if mag_x and mag_y:
        ax.set_xlim(min(mag_x) - 100, max(mag_x) + 100)
        ax.set_ylim(min(mag_y) - 100, max(mag_y) + 100)

    ax.scatter(mag_x, mag_y, s=2)

# === Save CSV when window is closed ===
def on_close(event):
    print("\nPlot window closed. Saving CSV...")
    with open(CSV_FILE, 'w') as f:
        f.write("x,y,z\n")  # Header
        for row in raw_data:
            f.write(','.join(row) + '\n')
    print(f"Saved {len(raw_data)} rows to {CSV_FILE}")

fig.canvas.mpl_connect('close_event', on_close)

# === Run Animation ===
anim = animation.FuncAnimation(fig, animate, interval=INTERVAL)
plt.show()
