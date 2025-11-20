import serial
import time
import matplotlib.pyplot as plt

# ------------------------
# User settings
# ------------------------
SERIAL_PORT = "/dev/cu.usbserial-0001"   # Change to your port
BAUDRATE = 115200

# ------------------------
# Open Serial
# ------------------------
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)

# Anchors in 2D (change these to your actual coordinates)
anchors = {
    "TAG1": (137.0, 0.0),
    "TAG2": (0.0, 0.0),
    "TAG3": (0.0, 127.0),
}

tag_pos = None

# ------------------------
# Setup matplotlib
# ------------------------
plt.ion()
fig, ax = plt.subplots()

def update_plot():
    ax.clear()

    # Plot anchors
    if anchors:
        xs = [anchors[k][0] for k in anchors]
        ys = [anchors[k][1] for k in anchors]

        ax.scatter(xs, ys, s=80, label="Anchors")
        for name, (x, y) in anchors.items():
            ax.text(x, y, name)

    # Plot tag position
    if tag_pos is not None:
        ax.scatter(tag_pos[0], tag_pos[1], s=120, label="Tag")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("UWB Anchor + Tag Visualization")
    ax.legend()
    ax.set_aspect("equal", "box")  # keep aspect ratio square

    plt.draw()
    plt.pause(0.01)

# ------------------------
# Main Loop
# ------------------------
print("Listening on serial...")

try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        # Example lines:
        #   "151.18,172.03"         -> we want to plot this
        #   "Distance (int): 44"    -> we want to ignore this
        #   "Triangulation failed..." -> ignore

        # Quick filter: if there's no comma, it can't be "x,y"
        if "," not in line:
            continue

        parts = line.split(",")
        if len(parts) != 2:
            continue  # skip weird lines

        try:
            x = float(parts[0])
            y = float(parts[1])
            tag_pos = (x, y)
            update_plot()
        except ValueError:
            # Not numeric "x,y" (e.g. some other line with a comma) → skip
            continue

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
