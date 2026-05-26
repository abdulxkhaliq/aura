from scservo_sdk import *

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000
SERVO_ID = 1

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_SPEED    = 46
ADDR_MODE          = 33

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_MODE, 1)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 1)

FULL_SPEED = 32767
packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_SPEED, FULL_SPEED)
print(f"🔄 Spinning at speed {FULL_SPEED} — press Ctrl+C to stop")

try:
    while True:
        pass

except KeyboardInterrupt:
    print("\nStopping...")
    packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_SPEED, 0)
    packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
    packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_MODE, 0)
    print("✅ Stopped — back to position mode")
    portHandler.closePort()
