from scservo_sdk import *
import time

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000

ADDR_LOCK    = 55
ADDR_ID      = 5

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

print("""
╔══════════════════════════════════╗
║     AURA Servo ID Fixer          ║
║                                  ║
║  Connect ONE servo at a time!    ║
╚══════════════════════════════════╝
""")

NEW_ID = int(input("Enter NEW ID for this servo (1-5): "))

print("\nSearching for servo...")
model, result, _ = packetHandler.ping(
    portHandler, BROADCAST_ID)
if result != COMM_SUCCESS:
    model, result, _ = packetHandler.ping(portHandler, 1)
    current_id = 1
else:
    current_id = BROADCAST_ID

if result != COMM_SUCCESS:
    print("❌ No servo found — check connection")
    exit()
print(f"✅ Servo found! Model: {model}")

print("Unlocking EEPROM...")
packetHandler.write1ByteTxRx(
    portHandler, current_id, ADDR_LOCK, 0)
time.sleep(0.1)

print(f"Writing ID {NEW_ID}...")
packetHandler.write1ByteTxRx(
    portHandler, current_id, ADDR_ID, NEW_ID)
time.sleep(0.5)

print("Locking EEPROM...")
packetHandler.write1ByteTxRx(
    portHandler, NEW_ID, ADDR_LOCK, 1)
time.sleep(0.1)

print("Verifying...")
model, result, _ = packetHandler.ping(portHandler, NEW_ID)
if result == COMM_SUCCESS:
    print(f"✅ SUCCESS! Servo ID={NEW_ID} confirmed!")
else:
    print(f"❌ Verification failed — try power cycling servo")

portHandler.closePort()
