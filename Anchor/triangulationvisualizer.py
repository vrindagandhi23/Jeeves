import serial
import time
import matplotlib.pyplot as plt

# ------------------------
# User settings
# ------------------------
SERIAL_PORT = "COM3"   # Change to your port
BAUDRATE = 115200

# ------------------------
# Open Serial
# ------------------------
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)

anchors = {
    "TAG1" : (0, 0),
    "TAG2" : (0, 0),
    "TAG3" : (0, 0)
}     # { "A0": (x,y,z), ... }
tag_pos = None

# ------------------------
# Setup matplotlib
# ------------------------
plt.ion()
fig = plt.figure()

ax = fig.add_subplot(111)

def update_plot():
    ax.clear()

    # Plot anchors
    if anchors:
        xs = [anchors[k][0] for k in anchors]
        ys = [anchors[k][1] for k in anchors]
        zs = [anchors[k][2] for k in anchors]

        ax.scatter(xs, ys, c='red', s=80, label="Anchors")
        for name, (x, y, z) in anchors.items():
            ax.text(x, y, name)

    # Plot tag position
    if tag_pos:
        ax.scatter(tag_pos[0], tag_pos[1], c='blue', s=120, label="Tag")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    ax.set_title("UWB Anchor + Tag Visualization")
    ax.legend()
    plt.draw()
    plt.pause(0.01)

# ------------------------
# Main Loop
# ------------------------
print("Listening on serial...")

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        continue
    
    tag_pos = tuple(map(float, line.split(",")))

    # Update plot
    update_plot()

