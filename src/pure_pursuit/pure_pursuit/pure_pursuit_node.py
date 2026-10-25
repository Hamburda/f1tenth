import os
import math
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter('csv_name','levine_centerline.csv')
        csv_name=self.get_parameter('csv_name').value
        self.waypoints=self.load_waypoints(csv_name)
        self.declare_parameter('lookahead',2.0)
        self.declare_parameter('speed',2.0)
        self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)
        self.drive_pub=self.create_publisher(AckermannDriveStamped,'/drive',10)
        # 2. încarcă traseul (apelează load_waypoints) și ține-l în self
        # 3. subscriber pe /ego_racecar/odom  -> odom_callback
        # 4. publisher pe /drive

    def load_waypoints(self, filename):
        folder = get_package_share_directory('pure_pursuit')
        path = os.path.join(folder, 'waypoints', filename)
        return np.loadtxt(path, delimiter=',', comments='#', usecols=(0, 1))

    def odom_callback(self, msg):
        # 1. scoate din msg: x, y și orientarea (yaw) a mașinii
        # 2. găsește punctul-țintă pe traseu (apelează find_goal)
        # 3. transformă punctul-țintă în coordonatele mașinii
        # 4. calculează curbura și unghiul de viraj
        # 5. publică pe /drive
        pass

    def find_goal(self, x, y):
        # alege din traseu punctul aflat la ~L metri în fața mașinii
        # întoarce coordonatele lui (pe hartă)
        pass


def main(args=None):
    # ca la toate nodurile: init, creează nodul, spin, shutdown
    pass