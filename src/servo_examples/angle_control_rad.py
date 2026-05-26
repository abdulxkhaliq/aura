from scservo_sdk import *
import math

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000
SERVO_ID = 5

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56
ADDR_MODE             = 33

TICKS_PER_RAD = 4096 / (2 * math.pi)
CENTER_TICK   = 2048

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_MODE, 0)
packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 1)

def rad_to_ticks(rad):
    return max(0, min(4095, int(CENTER_TICK + rad * TICKS_PER_RAD)))

def ticks_to_rad(ticks):
    return (ticks - CENTER_TICK) / TICKS_PER_RAD

def move_to(rad):
    ticks = rad_to_ticks(rad)
    packetHandler.write2ByteTxRx(portHandler, SERVO_ID, ADDR_GOAL_POSITION, ticks)
    print(f'-> {rad:.4f} rad  ({math.degrees(rad):.1f} deg)  ticks={ticks}')

def read_pos():
    ticks, result, _ = packetHandler.read2ByteTxRx(
        portHandler, SERVO_ID, ADDR_PRESENT_POSITION)
    if result == COMM_SUCCESS:
        rad = ticks_to_rad(ticks)
        print(f'current: {rad:.4f} rad  ({math.degrees(rad):.1f} deg)  ticks={ticks}')
    else:
        print('read failed')

print('\nAngle control — radians  (servo ID 6)')
print('  Enter radians, e.g.  1.92  or  -0.5')
print('  pos   → read current position')
print('  quit  → exit\n')

try:
    while True:
        cmd = input('rad> ').strip().lower()
        if cmd == 'quit':
            break
        elif cmd == 'pos':
            read_pos()
        else:
            try:
                move_to(float(cmd))
            except ValueError:
                print('enter a number in radians')
except KeyboardInterrupt:
    pass

packetHandler.write1ByteTxRx(portHandler, SERVO_ID, ADDR_TORQUE_ENABLE, 0)
portHandler.closePort()
