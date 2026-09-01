import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class AutoLevel(Node):
    def __init__(self):
        super().__init__('auto_level')

        self.publisher = self.create_publisher(
            Vector3,
            '/drone/cmd_attitude',
            10
        )

        # Publish at 20 Hz.
        # ESP32 watchdog timeout is 300 ms, so 50 ms updates
        # give plenty of margin.
        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info(
            'Auto-level active: roll=0, pitch=0, yaw_rate=0'
        )

    def update(self):
        msg = Vector3()

        # Desired LEVEL attitude
        msg.x = 0.0   # roll target, degrees
        msg.y = 0.0   # pitch target, degrees
        msg.z = 0.0   # yaw-rate target, deg/s

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AutoLevel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()