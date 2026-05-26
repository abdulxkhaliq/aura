from scservo_sdk import *

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000
SERVO_ID = 6

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56
ADDR_MODE             = 33

TICKS_PER_DEGREE = 4096 / 360

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_MODE, 0)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 1)
print("✅ Position mode enabled")

def angle_to_ticks(angle):
    ticks = int(angle * TICKS_PER_DEGREE)
    return max(0, min(4095, ticks))

def ticks_to_angle(ticks):
    return round(ticks / TICKS_PER_DEGREE, 2)

def move_to(angle):
    ticks = angle_to_ticks(angle)
    packetHandler.write2ByteTxRx(
        portHandler, SERVO_ID, ADDR_GOAL_POSITION, ticks)
    print(f"→ Moving to {angle}° (ticks={ticks})")

def read_angle():
    ticks, result, _ = packetHandler.read2ByteTxRx(
        portHandler, SERVO_ID, ADDR_PRESENT_POSITION)
    if result == COMM_SUCCESS:
        return ticks_to_angle(ticks)
    return None

print("\n🦾 Angle Controller")
print("─────────────────────────────")
print("  Enter angle: 0 - 360")
print("  Type 'pos'  to read current angle")
print("  Type 'quit' to exit")
print("─────────────────────────────\n")

try:
    while True:
        user_input = input("Enter angle: ").strip().lower()

        if user_input == 'quit':
            raise KeyboardInterrupt

        elif user_input == 'pos':
            angle = read_angle()
            if angle is not None:
                print(f"📍 Current angle: {angle}°")
            else:
                print("❌ Failed to read position")

        else:
            try:
                angle = float(user_input)
                if 0 <= angle <= 360:
                    move_to(angle)
                else:
                    print("⚠️  Please enter angle between 0 and 360")
            except ValueError:
                print("⚠️  Invalid input — enter a number like 90 or 180.5")

except KeyboardInterrupt:
    print("\nShutting down...")
    packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("✅ Done")
