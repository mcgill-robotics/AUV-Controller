import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Wrench


class WrenchTeleop(Node):
    def __init__(self):
        super().__init__('wrench_teleop')

        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.pub = self.create_publisher(
            Wrench,
            '/controls/effort',
            10
        )

        # Scaling factors (tune these)
        self.force_scale = 50.0     # Newtons
        self.torque_scale = 10.0    # Nm

        self.get_logger().info("Winmate Wrench Teleop started")

    def joy_callback(self, msg: Joy):
        wrench = Wrench()

        # ---- TRANSLATION (LEFT STICK) ----
        # axes indices may vary, adjust if needed
        forward = msg.axes[1]   # forward/back
        lateral = msg.axes[0]   # left/right
        vertical = msg.axes[4]  # up/down (trigger or stick)

        wrench.force.x = self.force_scale * forward
        wrench.force.y = self.force_scale * lateral
        wrench.force.z = self.force_scale * vertical

        # ---- ROTATION (RIGHT STICK) ----
        roll  = msg.axes[3]
        pitch = msg.axes[5]
        yaw   = msg.axes[2]

        wrench.torque.x = self.torque_scale * roll
        wrench.torque.y = self.torque_scale * pitch
        wrench.torque.z = self.torque_scale * yaw

        self.pub.publish(wrench)


def main():
    rclpy.init()
    node = WrenchTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

