#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Float64
from scservo_sdk import *
import math

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED    = 58

GRIPPER_ID = 6

ARM_SERVOS = {
    'shoulder_pan':  1,
    'shoulder_lift': 2,
    'elbow_flex':    3,
    'wrist_flex':    4,
    'wrist_roll':    5,
}
SERVOS = {**ARM_SERVOS, 'gripper': GRIPPER_ID}

TICKS_PER_RAD = 4096 / (2 * math.pi)
CENTER_TICK   = 2048

class ServoDriverNode(Node):
    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('port',              '/dev/ttyACM0')
        self.declare_parameter('baudrate',          1000000)
        self.declare_parameter('torque',            False)
        self.declare_parameter('gripper_open_rad',   1.74533)
        self.declare_parameter('gripper_close_rad', -0.174533)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        torque   = self.get_parameter('torque').value
        self.gripper_open_rad  = self.get_parameter('gripper_open_rad').value
        self.gripper_close_rad = self.get_parameter('gripper_close_rad').value

        self.portHandler   = PortHandler(port)
        self.packetHandler = PacketHandler(0)

        if not self.portHandler.openPort():
            self.get_logger().error(f'Cannot open {port}')
            return
        self.portHandler.setBaudRate(baudrate)
        self.get_logger().info(f'Connected on {port}')

        for name, sid in ARM_SERVOS.items():
            self.packetHandler.write1ByteTxRx(
                self.portHandler, sid, ADDR_TORQUE_ENABLE, 1 if torque else 0)
        arm_state = 'ON' if torque else 'OFF (backdrive)'
        self.get_logger().info(f'Arm torque: {arm_state}')

        self.packetHandler.write1ByteTxRx(
            self.portHandler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1)
        ticks = self.rad_to_ticks(self.gripper_open_rad)
        self.packetHandler.write2ByteTxRx(
            self.portHandler, GRIPPER_ID, ADDR_GOAL_POSITION, ticks)
        self.get_logger().info('Gripper torque: ON → open position')
        self.gripper_closed = False

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_subscription(
            Float64MultiArray, 'joint_commands', self.cmd_callback, 10)
        self.create_subscription(
            Float64MultiArray, 'torque_enable', self.torque_callback, 10)
        self.create_subscription(
            Bool, 'gripper_cmd', self.gripper_cmd_callback, 10)
        self.create_subscription(
            Float64, 'gripper_pos_cmd', self.gripper_pos_callback, 10)

        self.create_timer(0.02, self.read_and_publish)
        self.get_logger().info('Servo driver ready!')

    def rad_to_ticks(self, rad):
        return max(0, min(4095, int(CENTER_TICK + rad * TICKS_PER_RAD)))

    def ticks_to_rad(self, ticks):
        return (ticks - CENTER_TICK) / TICKS_PER_RAD

    def read_and_publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = []
        msg.position = []
        msg.velocity = []

        for name, sid in SERVOS.items():
            ticks, result, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, sid, ADDR_PRESENT_POSITION)
            if result == COMM_SUCCESS:
                msg.name.append(name)
                msg.position.append(self.ticks_to_rad(ticks))
                msg.velocity.append(0.0)

        self.joint_pub.publish(msg)

    def cmd_callback(self, msg):
        names = list(SERVOS.keys())
        for i, rad in enumerate(msg.data):
            if i >= len(names):
                break
            sid   = SERVOS[names[i]]
            ticks = self.rad_to_ticks(rad)
            self.packetHandler.write2ByteTxRx(
                self.portHandler, sid, ADDR_GOAL_POSITION, ticks)

    def torque_callback(self, msg):
        enable = int(msg.data[0])
        for _, sid in ARM_SERVOS.items():
            self.packetHandler.write1ByteTxRx(
                self.portHandler, sid, ADDR_TORQUE_ENABLE, enable)
        self.get_logger().info(f'Arm torque → {"ON" if enable else "OFF"}')

    def gripper_pos_callback(self, msg: Float64):
        lo = min(self.gripper_open_rad, self.gripper_close_rad)
        hi = max(self.gripper_open_rad, self.gripper_close_rad)
        rad = max(lo, min(hi, msg.data))
        ticks = self.rad_to_ticks(rad)
        self.packetHandler.write2ByteTxRx(
            self.portHandler, GRIPPER_ID, ADDR_GOAL_POSITION, ticks)

    def gripper_cmd_callback(self, msg: Bool):
        self.gripper_closed = msg.data
        rad   = self.gripper_close_rad if msg.data else self.gripper_open_rad
        ticks = self.rad_to_ticks(rad)
        self.packetHandler.write2ByteTxRx(
            self.portHandler, GRIPPER_ID, ADDR_GOAL_POSITION, ticks)
        state = 'CLOSED' if msg.data else 'OPEN'
        self.get_logger().info(f'Gripper → {state} ({rad:.2f} rad)')

    def destroy_node(self):
        for _, sid in SERVOS.items():
            self.packetHandler.write1ByteTxRx(
                self.portHandler, sid, ADDR_TORQUE_ENABLE, 0)
        self.portHandler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
