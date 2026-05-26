from scservo_sdk import *
import time

PORT     = '/dev/ttyACM0'

portHandler = PortHandler(PORT)

BAUDRATES = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]
PROTOCOLS = [0, 1]

for protocol in PROTOCOLS:
    packetHandler = PacketHandler(protocol)
    for baud in BAUDRATES:
        try:
            portHandler.openPort()
            portHandler.setBaudRate(baud)
            
            for sid in range(0, 20):
                model, result, error = packetHandler.ping(portHandler, sid)
                if result == COMM_SUCCESS:
                    print(f"✅ FOUND! Protocol={protocol} Baud={baud} ID={sid} Model={model}")
            
            portHandler.closePort()
            time.sleep(0.1)
        except Exception as e:
            print(f"Error: {e}")

print("Scan complete")
