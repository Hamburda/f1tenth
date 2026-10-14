import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.speed = 0.0
        self.ttc_threshold = 1.0
        self.braking = False 

        self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped,'drive',10)

    def odom_callback(self,msg):
        self.speed=msg.twist.twist.linear.x

    def scan_callback(self,msg):
        ranges=np.array(msg.ranges)
        angles= msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        range_rate = self.speed *np.cos(angles)
        valid = np.isfinite(ranges) & (ranges>msg.range_min) &(range_rate >0)
        ttc = np.full(len(ranges),np.inf)
        ttc[valid] = ranges[valid] /range_rate[valid]

        if not self.braking and np.min(ttc) < self.ttc_threshold:
            self.braking = True
            self.get_logger().info(f'FRANA! iTTC minim = {np.min(ttc):.2f} s')

        if self.braking:
            brake = AckermannDriveStamped()
            brake.drive.speed = 0.0
            self.drive_pub.publish(brake)

            if abs(self.speed) < 0.05:
                self.braking = False
                self.get_logger().info('Oprit. Frana eliberata.')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()