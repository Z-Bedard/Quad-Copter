import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class AutoDemo(Node):
    def __init__(self):
        super().__init__('auto_demo')

        self.publisher = self.create_publisher(
            Vector3,
            '/drone/cmd_attitude',
            10
        )

        # roll, pitch, yaw_rate, duration seconds
        self.sequence = [
            ( 0.0,  0.0, 0.0, 2.0),
            ( 3.0,  0.0, 0.0, 2.0),
            ( 0.0,  0.0, 0.0, 2.0),
            (-3.0,  0.0, 0.0, 2.0),
            ( 0.0,  0.0, 0.0, 2.0),
            ( 0.0,  3.0, 0.0, 2.0),
            ( 0.0,  0.0, 0.0, 2.0),
            ( 0.0, -3.0, 0.0, 2.0),
            ( 0.0,  0.0, 0.0, 3.0),
        ]

        self.step = 0
        self.step_start = self.get_clock().now()

        # 20 Hz keeps your 300 ms FC watchdog satisfied
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        if self.step >= len(self.sequence):
            roll = 0.0
            pitch = 0.0
            yaw_rate = 0.0
        else:
            roll, pitch, yaw_rate, duration = self.sequence[self.step]

            elapsed = (
                self.get_clock().now() - self.step_start
            ).nanoseconds / 1e9

            if elapsed >= duration:
                self.step += 1
                self.step_start = self.get_clock().now()
                return

        msg = Vector3()
        msg.x = roll
        msg.y = pitch
        msg.z = yaw_rate

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
