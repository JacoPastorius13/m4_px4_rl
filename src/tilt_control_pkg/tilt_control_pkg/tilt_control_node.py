import rclpy
from rclpy.node import Node
from custom_msgs.msg import TiltCommand

class TiltCommandNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.publisher = self.create_publisher(TiltCommand, '/fmu/in/tilt_command', qos_profile)
        self.publish_tilt_command()

    def publish_tilt_command(self):
        msg = TiltCommand()
        msg.timestamp = self.get_clock().now().nanoseconds  # Use nanoseconds for uint64
        msg.tilt_command = 0.5
        self.publisher.publish(msg)
        self.get_logger().info(f'Published tilt control command: {msg.tilt_command}')

def main(args=None):
    rclpy.init(args=args)
    tilt_command_node = TiltCommandNode()

    try:
        rclpy.spin(tilt_command_node)
    finally:
        tilt_command_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()