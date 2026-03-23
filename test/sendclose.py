# import serial
# import time

# port = "/dev/ttyACM2"
# baud = 115200

# with serial.Serial(port, baud, timeout=1) as ser:
#     time.sleep(0.1)  # let port settle

#     ser.write(b"C\r")
#     print(f"Sent: C\\r")
#     time.sleep(0.1)

#     # ser.write(b"S5\r")
#     # print(f"Sent: S5\\r")
#     # time.sleep(0.1)
    
#     # ser.write(b"O\r")
#     # print(f"Sent: O\\r")
#     # time.sleep(0.1)


#     resp = ser.read(ser.in_waiting or 1)
#     print(f"Response: {resp.hex()}")


import serial
import time
import subprocess
import os

port = "/dev/ttyACM3"
baud = 115200

# Step 1: bring down can0
print("Bringing down can0...")
os.system("sudo ip link set can0 down")
time.sleep(0.2)

# Step 2: kill slcand
print("Killing slcand...")
os.system("sudo pkill slcand")
time.sleep(0.3)

# Step 3: send C\r directly to the port
print("Sending close command to ESP32...")
with serial.Serial(port, baud, timeout=1) as ser:
    time.sleep(0.1)
    ser.write(b"C\r")
    time.sleep(0.1)
    resp = ser.read(ser.in_waiting or 1)
    status = "OK \\r" if resp == b"\r" else f"got {resp.hex()}"
    print(f"Close: sent b'C\\r' → {status}")

print("Done.")