from scservo_sdk import *

PORT = '/dev/ttyACM0'
BAUD = 1000000

OLD_ID = 1
NEW_ID = 6

ADDR_ID = 5

port = PortHandler(PORT)
packet = PacketHandler(0)

port.openPort()
port.setBaudRate(BAUD)

packet.write1ByteTxRx(port, OLD_ID, 40, 0)

packet.write1ByteTxRx(port, OLD_ID, ADDR_ID, NEW_ID)

print(f"✅ Changed ID {OLD_ID} → {NEW_ID}")

port.closePort()
