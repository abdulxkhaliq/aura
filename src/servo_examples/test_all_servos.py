from scservo_sdk import *
import time

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000

SERVOS = {
    'shoulder_pan':  1,
    'shoulder_lift': 2,
    'elbow_flex':    3,
    'wrist_flex':    4,
    'wrist_roll':    5,
}

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

print("=== AURA Servo Test ===\n")
all_ok = True
for name, sid in SERVOS.items():
    model, result, _ = packetHandler.ping(portHandler, sid)
    if result == COMM_SUCCESS:
        pos, _, _ = packetHandler.read2ByteTxRx(
            portHandler, sid, ADDR_PRESENT_POSITION)
        print(f"✅ {name:20s} ID={sid} pos={pos}")
    else:
        print(f"❌ {name:20s} ID={sid} NOT FOUND")
        all_ok = False

if all_ok:
    print("\n✅ All servos OK! Moving to home position...")
    for name, sid in SERVOS.items():
        packetHandler.write1ByteTxRx(
            portHandler, sid, ADDR_TORQUE_ENABLE, 1)
        packetHandler.write2ByteTxRx(
            portHandler, sid, ADDR_GOAL_POSITION, 2048)
    time.sleep(3)
    for name, sid in SERVOS.items():
        packetHandler.write1ByteTxRx(
            portHandler, sid, ADDR_TORQUE_ENABLE, 0)
    print("✅ Home position set — torque disabled")
else:
    print("\n❌ Fix missing servos before continuing")

portHandler.closePort()
