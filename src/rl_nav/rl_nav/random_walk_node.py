import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RandomWalkNode(Node):
    """
    Απλός τοπικός controller:
    - Διαβάζει LaserScan.
    - Αν υπάρχει εμπόδιο μπροστά σε απόσταση < threshold → στρίβει.
    - Αλλιώς → προχωράει μπροστά.
    """

    def __init__(self):
        super().__init__('random_walk_node')

        # Publisher για ταχύτητες
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber για LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Παράμετροι
        self.declare_parameter('forward_speed', 0.15)   # m/s
        self.declare_parameter('turn_speed', 0.6)       # rad/s
        self.declare_parameter('obstacle_distance', 0.5)  # m

        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.obstacle_distance = self.get_parameter('obstacle_distance').get_parameter_value().double_value

        self.get_logger().info('RandomWalkNode started.')

    def scan_callback(self, msg: LaserScan):
        """
        Εδώ παίρνουμε το LaserScan και αποφασίζουμε κίνηση.
        Θα κοιτάμε ένα "παράθυρο" μπροστά (π.χ. ±30°) και θα βρίσκουμε τη min απόσταση.
        """

        n = len(msg.ranges)
        if n == 0:
            return

        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        # Μπροστά περίπου [-30°, +30°]
        front_min_angle = -math.radians(30)
        front_max_angle = math.radians(30)

        front_indices = []
        for i in range(n):
            angle = angle_min + i * angle_increment
            if front_min_angle <= angle <= front_max_angle:
                front_indices.append(i)

        if not front_indices:
            front_indices = range(n)

        front_distances = []
        for i in front_indices:
            d = msg.ranges[i]
            if math.isfinite(d):
                front_distances.append(d)

        if not front_distances:
            self.stop_robot()
            return

        min_front_dist = min(front_distances)

        twist = Twist()

        if min_front_dist < self.obstacle_distance:
            # Πολύ κοντά σε εμπόδιο → στρίψε
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.get_logger().debug(
                f'Obstacle detected at {min_front_dist:.2f} m → turning.'
            )
        else:
            # Καθαρό μπροστά → προχώρα
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            self.get_logger().debug(
                f'Path clear, min_front_dist={min_front_dist:.2f} m → moving forward.'
            )

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
