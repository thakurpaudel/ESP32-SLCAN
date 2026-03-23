import can
from can.interfaces.slcan import slcanBus

bus = slcanBus(
    channel="/dev/ttyACM0",
    bitrate=250000,
    sleep_after_open=2,
    rtscts=False,
    listen_only=False,
)

print("Listening on CAN bus... Press Ctrl-C to stop.\n")
try:
    while True:
        msg = bus.recv()
        print(msg)
finally:
    print("Closing the bus")
    # bus.shutdown()
