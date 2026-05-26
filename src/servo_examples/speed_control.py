from scservo_sdk import *
import sys
import tty
import termios

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

speed = 0
STEP  = 1000
MAX   = 32767

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def set_speed(s):
    packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_SPEED, s)

print("🔄 Speed Controller")
print("─────────────────────────────")
print("  ↑ or W  = increase speed")
print("  ↓ or S  = decrease speed")
print("  R       = reverse direction")
print("  SPACE   = stop")
print("  Q       = quit")
print("─────────────────────────────")
print(f"Current speed: {speed}")

direction = 1

try:
    while True:
        key = get_key()

        if key in ('w', 'W', '\x1b'):
            if key == '\x1b':
                sys.stdin.read(1)
                arrow = sys.stdin.read(1)
                if arrow == 'A':
                    key = 'w'
                elif arrow == 'B':
                    key = 's'

            if key in ('w', 'W'):
                speed = min(speed + STEP, MAX)
                actual = speed if direction == 1 else speed | 0x8000
                set_speed(actual)
                print(f"⬆️  Speed: {speed} | Direction: {'CW' if direction==1 else 'CCW'}")

        elif key in ('s', 'S'):
            speed = max(speed - STEP, 0)
            actual = speed if direction == 1 else speed | 0x8000
            set_speed(actual)
            print(f"⬇️  Speed: {speed} | Direction: {'CW' if direction==1 else 'CCW'}")

        elif key in ('r', 'R'):
            direction *= -1
            actual = speed if direction == 1 else speed | 0x8000
            set_speed(actual)
            print(f"🔄 Reversed! Direction: {'CW' if direction==1 else 'CCW'} Speed: {speed}")

        elif key == ' ':
            speed = 0
            set_speed(0)
            print("⏹️  Stopped")

        elif key in ('q', 'Q'):
            raise KeyboardInterrupt

except KeyboardInterrupt:
    print("\n\nShutting down...")
    set_speed(0)
    packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
    packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_MODE, 0)
    portHandler.closePort()
    print("✅ Done")
