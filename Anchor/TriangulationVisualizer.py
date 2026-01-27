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
    "TAG4": (137.0, 127.0)
}

tag_pos = None

# Precompute axis limits so the plot doesn't keep rescaling
anchor_xs = [p[0] for p in anchors.values()]
anchor_ys = [p[1] for p in anchors.values()]
margin = 20  # cm padding around anchors

X_MIN = min(anchor_xs) - margin
X_MAX = max(anchor_xs) + margin
Y_MIN = min(anchor_ys) - margin
Y_MAX = max(anchor_ys) + margin

# ------------------------
# Setup matplotlib
# ------------------------
plt.ion()
fig, ax = plt.subplots()

def update_plot():
    ax.clear()

    # Fix the axis limits so the view doesn't jump around
    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)

    # Plot anchors
    xs = [anchors[k][0] for k in anchors]
    ys = [anchors[k][1] for k in anchors]
    ax.scatter(xs, ys, s=80, label="Anchors")
    for name, (x, y) in anchors.items():
        ax.text(x, y, name)

    # Plot tag position
    if tag_pos is not None:
        ax.scatter(tag_pos[0], tag_pos[1], s=120, label="Tag")

    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_title("UWB Anchor + Tag Visualization")
    ax.legend()
    ax.set_aspect("equal", "box")  # keep aspect ratio square

    plt.draw()
    plt.pause(0.001)

# ------------------------
# Main Loop
# ------------------------
print("Listening on serial...")

try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

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
