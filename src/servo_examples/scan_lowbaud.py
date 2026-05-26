from scservo_sdk import *
import time

PORT = '/dev/ttyACM0'
BAUDRATES = [115200, 500000, 250000, 1000000]

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)
portHandler.openPort()

for baud in BAUDRATES:
    portHandler.setBaudRate(baud)
    print(f"\nTrying {baud} baud...")
    for sid in range(1, 6):
        model, result, _ = packetHandler.ping(portHandler, sid)
        if result == COMM_SUCCESS:
            print(f"  ✅ ID {sid} found at {baud} baud!")
        else:
            print(f"  ❌ ID {sid} no response")
    time.sleep(0.2)

portHandler.closePort()
