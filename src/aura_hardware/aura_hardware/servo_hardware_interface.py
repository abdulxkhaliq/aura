#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scservo_sdk import *
import math

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56

SERVOS = {
    'shoulder_pan':  1,
    'shoulder_lift': 2,
    'elbow_flex':    3,
    'wrist_flex':    4,
    'wrist_roll':    5,
    'gripper':       6,
}

TICKS_PER_RAD = 4096 / (2 * math.pi)
CENTER_TICK   = 2048

class ServoHardwareInterface(Node):
    def __init__(self):
        super().__init__('servo_hardware_interface')

        self.declare_parameter('port',     '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.portHandler   = PortHandler(port)
        self.packetHandler = PacketHandler(0)

        if not self.portHandler.openPort():
            self.get_logger().error(f'Cannot open {port}')
            return
        self.portHandler.setBaudRate(baudrate)
        self.get_logger().info(f'✅ Connected on {port}')

        for name, sid in SERVOS.items():
            self.packetHandler.write1ByteTxRx(
                self.portHandler, sid,
                ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info('✅ Torque enabled on all servos')

        self.joint_pub = self.create_publisher(
            JointState, 'joint_states', 10)

        self.create_subscription(
            Float64MultiArray,
            '/arm_controller/commands',
            self.cmd_callback, 10)

        from trajectory_msgs.msg import JointTrajectory
        self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback, 10)

        self.create_timer(0.02, self.read_and_publish)
        self.get_logger().info('✅ Hardware interface ready!')

    def rad_to_ticks(self, rad):
        return max(0, min(4095,
            int(CENTER_TICK + rad * TICKS_PER_RAD)))

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
            else:
                self.get_logger().warn(
                    f'Read failed: {name}',
                    throttle_duration_sec=2.0)

        self.joint_pub.publish(msg)

    def cmd_callback(self, msg):
        names = list(SERVOS.keys())
        for i, rad in enumerate(msg.data):
            if i >= len(names):
                break
            sid   = SERVOS[names[i]]
            ticks = self.rad_to_ticks(rad)
            self.packetHandler.write2ByteTxRx(
                self.portHandler, sid,
                ADDR_GOAL_POSITION, ticks)

    def trajectory_callback(self, msg):
        if not msg.points:
            return
        point  = msg.points[-1]
        names  = msg.joint_names
        for i, name in enumerate(names):
            if name in SERVOS:
                sid   = SERVOS[name]
                rad   = point.positions[i]
                ticks = self.rad_to_ticks(rad)
                self.packetHandler.write2ByteTxRx(
                    self.portHandler, sid,
                    ADDR_GOAL_POSITION, ticks)

    def destroy_node(self):
        for _, sid in SERVOS.items():
            self.packetHandler.write1ByteTxRx(
                self.portHandler, sid,
                ADDR_TORQUE_ENABLE, 0)
        self.portHandler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoHardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
