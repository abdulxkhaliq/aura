from scservo_sdk import *
import time

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000
SERVO_ID = 1

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED    = 46
ADDR_MODE          = 33

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)
print("✅ Connected!")

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
print("✅ Torque disabled")

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_MODE, 1)
print("✅ Wheel mode set")

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 1)
print("✅ Torque enabled")

SPEED_CW  = 500
SPEED_CCW = 500 | 0x8000

print("\nSpinning clockwise for 3 seconds...")
packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_SPEED, SPEED_CW)
time.sleep(3)

print("Spinning counterclockwise for 3 seconds...")
packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_SPEED, SPEED_CCW)
time.sleep(3)

print("Stopping...")
packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_SPEED, 0)

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_MODE, 0)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 1)
print("✅ Back to position mode")

portHandler.closePort()
