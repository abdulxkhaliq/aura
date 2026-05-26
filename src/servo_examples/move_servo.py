from scservo_sdk import *
import time

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000
SERVO_ID = 1

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED    = 58

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)
print("✅ Connected!")

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 1)
print("✅ Torque enabled")

positions = [2048, 1024, 3072, 2048]
labels    = ["center", "left", "right", "center"]

for pos, label in zip(positions, labels):
    print(f"\nMoving to {label} (ticks={pos})...")
    packetHandler.write2ByteTxRx(
        portHandler, SERVO_ID, ADDR_GOAL_POSITION, pos)
    time.sleep(2)

    current, result, _ = packetHandler.read2ByteTxRx(
        portHandler, SERVO_ID, ADDR_PRESENT_POSITION)
    if result == COMM_SUCCESS:
        print(f"  Current position: {current} ticks")

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
print("\n✅ Done — torque disabled")

portHandler.closePort()
