from scservo_sdk import *
import time

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42

HOME_POSITIONS = {
    1: 2048,
    2: 2048,
    3: 2048,
    4: 2048,
    5: 2048,
}

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

print("Moving all servos to HOME (zero) position...")
print("⚠️  Make sure arm has clearance to move!\n")
input("Press ENTER to confirm...")

for sid in HOME_POSITIONS:
    packetHandler.write1ByteTxRx(
        portHandler, sid, ADDR_TORQUE_ENABLE, 1)

for sid, ticks in HOME_POSITIONS.items():
    packetHandler.write2ByteTxRx(
        portHandler, sid, ADDR_GOAL_POSITION, ticks)
    print(f"✅ Servo {sid} → center (2048 ticks)")
    time.sleep(0.1)

print("\n✅ All servos at HOME position")
print("This is your ZERO position for MoveIt2")
time.sleep(3)

for sid in HOME_POSITIONS:
    packetHandler.write1ByteTxRx(
        portHandler, sid, ADDR_TORQUE_ENABLE, 0)

portHandler.closePort()
