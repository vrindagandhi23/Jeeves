# ESP32 UWB

## UWB Settings/Setup
### Settings: 
Network: JEEVES

Tag Addresses: 
TAG1
TAG2
TAG3

Anchors Addresses:
ANCHOR1

CPIN:00000000000000000000000000000000

### Setup:
Flash ATCommand.ino on the ESP32 and write to the serial monitor the following commands. After each command, you should be getting a response. 
#### Steps:
Designate Tag Type (one or the other):

AT+MODE=0 (tag), AT+MODE=1 (anchor)

AT+NETWORKID=JEEVES

AT+ADDRESS={Address}

AT+CPIN=00000000000000000000000000000000

#### Debug
If not getting a response after performing the commands, then just send

AT

and you should recieve an OK response. If no response, check ESP32 wiring and power.

## Program Functions
### ATCommand.ino
Used to send and recieve AT commands to ESP32
### AnchorPolling.ino
Flash the code onto the ESP32 connected to the Anchor. It will poll the 3 Tags and print their distance from the Anchor.
### Triangulation.ino
Built off AnchorPolling.ino. Polls the 3 distances from the tags and based on a given position for these anchors (specificed in the file), prints the position of the anchor.
### TriangulationVisualizer.py
Runs on the computer connected to the Anchor running Triangulation.ino. Visualizes the position of the tags and anchor in real time.

**To visualize:** (1) Upload **Triangulation.ino** to the ESP32 (not AnchorPolling—that one only prints distances). (2) Close the Arduino Serial Monitor. (3) Run `python TriangulationVisualizer.py` from the `Anchor` folder. The script will open the serial port, show the plot, and print `Position: x, y` in the terminal so you don't need the Serial Monitor open.



