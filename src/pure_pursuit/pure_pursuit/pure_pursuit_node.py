import os
import math
import rclpy
import numpy as np
from rclpy.node import Node
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
        self.lookahead=self.get_parameter('lookahead').value
        self.speed=self.get_parameter('speed').value
        self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)
        self.drive_pub=self.create_publisher(AckermannDriveStamped,'/drive',10)

        
    def load_waypoints(self, filename):
        folder = get_package_share_directory('pure_pursuit')
        path = os.path.join(folder, 'waypoints', filename)
        return np.loadtxt(path, delimiter=',', comments='#', usecols=(0, 1))

    def odom_callback(self, msg):
        # 1. scoate din msg: x, y și orientarea (yaw) a mașinii
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        qx=msg.pose.pose.orientation.x
        qy=msg.pose.pose.orientation.y
        qz=msg.pose.pose.orientation.z
        qw=msg.pose.pose.orientation.w
        yaw= math.atan2(2*(qw*qz + qx*qy),1-2*(qy*qy +qz*qz))
        
        # 2. găsește punctul-țintă pe traseu (apelează find_goal)
        gx, gy = self.find_goal(x,y)
        # 3. transformă punctul-țintă în coordonatele mașinii
        dx=gx-x
        dy=gy-y
        theta=-yaw
        [x_l,y_l] = [math.cos(theta)*dx  - math.sin(theta)*dy, math.sin(theta)*dx + math.cos(theta)*dy]
        # 4. calculează curbura și unghiul de viraj
        L=x_l*x_l + y_l*y_l
        Y=2*y_l/L
        ampartament = 0.33
        delta=math.atan(ampartament*Y)
        delta= np.clip(delta,-0.4189,0.4189)
        # 5. publică pe /drive
        drive= AckermannDriveStamped()
        drive.drive.steering_angle = delta
        drive.drive.speed= self.speed
        self.drive_pub.publish(drive)

    def find_goal(self, x, y):
        # alege din traseu punctul aflat la ~L metri în fața mașinii
        A=self.waypoints - np.array([x,y])
        v_dist=np.linalg.norm(A,axis=1)
        i=np.argmin(v_dist)
        while v_dist[i]<=self.lookahead :
            i=(i+1) % len(self.waypoints)
        # întoarce coordonatele lui (pe hartă)
        return self.waypoints[i]


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()