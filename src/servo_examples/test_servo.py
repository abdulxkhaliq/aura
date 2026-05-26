from scservo_sdk import *

PORT     = '/dev/ttyACM0'
BAUDRATE = 1000000
SERVO_ID = 1

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(0)

if not portHandler.openPort():
    print("❌ Failed to open port — check USB connection")
    exit()
print("✅ Port opened")

portHandler.setBaudRate(BAUDRATE)
print("✅ Baudrate set")

print(f"Pinging servo ID {SERVO_ID}...")
model, result, error = packetHandler.ping(portHandler, SERVO_ID)

if result == COMM_SUCCESS:
    print(f"✅ Servo found! Model: {model}")
else:
    print(f"❌ No response. result={result}, error={error}")
    print("→ Check power supply is ON")
    print("→ Check TTL cable is plugged into servo")

portHandler.closePort()
