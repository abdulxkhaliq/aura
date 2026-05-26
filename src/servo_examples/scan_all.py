from scservo_sdk import *

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

print("Scanning IDs 0-20...\n")
found = []
for sid in range(0, 21):
    model, result, error = packetHandler.ping(portHandler, sid)
    if result == COMM_SUCCESS:
        print(f"✅ Found servo at ID {sid} model={model}")
        found.append(sid)

if not found:
    print("❌ No servos found at all")
    print("\nPossible issues:")
    print("1. Power supply not ON")
    print("2. Wrong port — check ls /dev/ttyACM*")
    print("3. Cable not in TTL port on board")
else:
    print(f"\nFound {len(found)} servo(s) at IDs: {found}")
    print("Likely all on default ID 1 — need to assign IDs")

portHandler.closePort()
