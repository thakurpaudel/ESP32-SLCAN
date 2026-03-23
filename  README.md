ESP32-SLCAN
Turn your ESP32 into a USB CAN adapter using SLCAN protocol.
Hardware
CAN Transceiver Wiring:
ESP32 GPIO 21 → CAN TX
ESP32 GPIO 22 → CAN RX
ESP32 3.3V    → CAN VCC
ESP32 GND     → CAN GND
Build & Flash
bash. $HOME/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash
Usage
Linux
bash# Install tools
sudo apt-get install can-utils

# Setup interface using UART(/dev/ttyUSBx)
```
sudo slcand -o -c -s5 -S 115200 /dev/ttyUSBx can0
sudo ip link set up can0
```

# Setup interface using USB(/dev/ttyACMx)
```
sudo slcand -o -c -s5 /dev/ttyACMx can0
sudo ip link set up can0
```

# Listen
candump can0

# Send
cansend can0 123#DEADBEEF
Python
bashpip install python-can
pythonimport can

# Open bus
bus = can.Bus(channel='/dev/ttyUSB0', interface='slcan', bitrate=500000)

# Send message
msg = can.Message(arbitration_id=0x123, data=[0xDE, 0xAD, 0xBE, 0xEF])
bus.send(msg)

# Receive messages
while True:
    msg = bus.recv()
    print(f"ID: {msg.arbitration_id:X} Data: {msg.data.hex()}")
macOS
bashpip3 install python-can

# Find port
ls /dev/cu.usbserial-*

# Use with python-can
python3 -m can.logger -i slcan -c /dev/cu.usbserial-XXXX -b 500000
Bitrates
CodeBitrateS4125 kbit/sS5250 kbit/sS6500 kbit/s (default)S81 Mbit/s
License
MIT