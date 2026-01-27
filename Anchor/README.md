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
### Polling.ino



