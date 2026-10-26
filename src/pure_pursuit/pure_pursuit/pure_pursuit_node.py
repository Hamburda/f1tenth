import os
import math
import rclpy
import numpy as np
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter('csv_name','levine_centerline.csv')
        csv_name=self.get_parameter('csv_name').value
        self.waypoints=self.load_waypoints(csv_name)
        self.declare_parameter('speed',2.0)
        self.speed=self.get_parameter('speed').value
        self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)
        self.drive_pub=self.create_publisher(AckermannDriveStamped,'/drive',10)
        # markere pentru Foxglove: traseul (republicat la 1 s) și ținta curentă
        self.marker_pub=self.create_publisher(Marker,'/pure_pursuit/markers',10)
        self.path_marker=self.make_path_marker()
        self.create_timer(1.0,self.publish_path)
        self.declare_parameter('vmax',8.0)
        self.declare_parameter('vmin',4.0)
        self.declare_parameter('a_lat',6.0)
        self.declare_parameter('a_brake',6.0)
        self.vmax=self.get_parameter('vmax').value
        self.vmin=self.get_parameter('vmin').value
        self.a_lat=self.get_parameter('a_lat').value
        self.a_brake=self.get_parameter('a_brake').value
        self.declare_parameter('a_acc',3.0)
        self.a_acc=self.get_parameter('a_acc').value
        self.declare_parameter('k',0.25)
        self.k=self.get_parameter('k').value
        self.declare_parameter('L_min',1.5)
        self.L_min=self.get_parameter('L_min').value
        self.declare_parameter('L_max',4.0)
        self.L_max=self.get_parameter('L_max').value
        self.declare_parameter('L0',0.0)
        self.L0=self.get_parameter('L0').value


        #mapa viteza
        self.kappa=self.compute_curvature()
        self.get_logger().info(f'kappa min {self.kappa.min():.3f} max {self.kappa.max():.3f}')
        self.v_map=self.accel_profile(self.brake_profile(self.turn_speed()))
        self.get_logger().info(f'v_map min {self.v_map.min():.3f} max {self.v_map.max():.3f}')

    def accel_profile(self,v):
        v=v.copy()
        N=len(v)
        for _ in range(2):
            for i in range(N):
                j=(i+1)%N
                d= np.linalg.norm(self.waypoints[i] - self.waypoints[j])
                v[j]=min(v[j],np.sqrt(v[i]*v[i] + 2*self.a_acc*d))
        return v
    def brake_profile(self,v):
        v=v.copy()
        N=len(v)
        for _ in range(2):
            for i in range(N-1,-1,-1):
                j=(i+1)%N
                d= np.linalg.norm(self.waypoints[j] - self.waypoints[i])
                v[i]=min(v[i],np.sqrt(v[j]*v[j] + 2*self.a_brake*d))
        return v
    def turn_speed(self):
        v=np.sqrt(self.a_lat/(self.kappa + 1e-6))
        v=np.clip(v,self.vmin,self.vmax)
        return v
        

    def compute_curvature(self):
        wp=self.waypoints
        p0=np.roll(wp,1,axis=0)
        p1=wp
        p2=np.roll(wp,-1,axis= 0)

        a=np.linalg.norm(p1-p0,axis=1)
        b=np.linalg.norm(p2-p1,axis=1)
        c=np.linalg.norm(p0-p2,axis=1)

        u= p1-p0
        w=p2-p0
        aria = 0.5 * (np.abs(u[:,0]*w[:,1] - u[:,1] * w[:,0]))

        kappa = 4*aria/(a*b*c +1e-9)
        return kappa
    
    def make_marker(self, marker_id, marker_type, r, g, b):
        m=Marker()
        m.header.frame_id='map'
        m.ns='pure_pursuit'
        m.id=marker_id
        m.type=marker_type
        m.action=Marker.ADD
        m.pose.orientation.w=1.0
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        return m

    def make_path_marker(self):
        m=self.make_marker(0,Marker.LINE_STRIP,0.0,0.6,1.0)
        m.scale.x=0.05
        m.points=[Point(x=float(px),y=float(py),z=0.0) for px,py in self.waypoints]
        m.points.append(m.points[0])  # închide bucla
        return m

    def publish_path(self):
        self.path_marker.header.stamp=self.get_clock().now().to_msg()
        self.marker_pub.publish(self.path_marker)

    def publish_goal(self, gx, gy):
        m=self.make_marker(1,Marker.SPHERE,1.0,0.2,0.2)
        m.header.stamp=self.get_clock().now().to_msg()
        m.scale.x=m.scale.y=m.scale.z=0.25
        m.pose.position.x=float(gx)
        m.pose.position.y=float(gy)
        self.marker_pub.publish(m)

    def load_waypoints(self, filename):
        folder = get_package_share_directory('pure_pursuit')
        path = os.path.join(folder, 'waypoints', filename)
        # raceline-urile (format TUM) au ';' și coloanele s, x, y, ...; centerline-urile au ',' și x, y, ...
        with open(path) as f:
            first_data = next(line for line in f if not line.startswith('#'))
        raceline = ';' in first_data
        if raceline:
            return np.loadtxt(path, delimiter=';', comments='#', usecols=(1, 2))
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
        curr_speed=msg.twist.twist.linear.x
        look=np.clip(curr_speed*self.k + self.L0,self.L_min,self.L_max)
        # 2. găsește punctul-țintă pe traseu (apelează find_goal)
        (gx, gy), i_near = self.find_goal(x,y,look)
        self.publish_goal(gx, gy)
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
        drive.drive.speed= self.v_map[i_near]
        self.drive_pub.publish(drive)

    def find_goal(self, x, y,l):
        # alege din traseu punctul aflat la ~L metri în fața mașinii
        A=self.waypoints - np.array([x,y])
        v_dist=np.linalg.norm(A,axis=1)
        i=np.argmin(v_dist)
        i_near=np.argmin(v_dist)
        while v_dist[i]<= l :
            i=(i+1) % len(self.waypoints)
        # întoarce coordonatele lui (pe hartă)
        return self.waypoints[i], i_near


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()