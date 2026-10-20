import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap(Node):
    def param(self,name,default):
        self.declare_parameter(name,default)
        return self.get_parameter(name).value
    def __init__(self):
        super().__init__('reactive_node')
        self.create_subscription(LaserScan,'/scan',self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,'/drive', 10)
        self.fov = math.radians(self.param('fov_deg', 90.0))
        self.max_range = self.param('max_range', 3.0)
        self.smooth_window = self.param('smooth_window', 5)
        self.disparity_threshold = self.param('disparity_threshold', 0.3)
        self.car_half_width = self.param('car_half_width', 0.25)
        self.bubble_radius = self.param('bubble_radius', 0.2)
        self.deep_ratio = self.param('deep_ratio', 0.9)
        self.max_steer = self.param('max_steer', 0.4)
        self.steer_alpha = self.param('steer_alpha', 0.1)
        self.max_speed = self.param('max_speed', 3.0)
        self.min_speed = self.param('min_speed', 1.0)

        self.steer = 0.0

    def preprocess_lidar(self, ranges, angles):
        mask=np.abs(angles) <= self.fov
        r = ranges[mask].copy()
        a= angles[mask]

        r[np.isposinf(r)] =self.max_range
        r[~np.isfinite(r)] =0.0
        r = np.clip(r, 0.0, self.max_range)

        k=self.smooth_window
        # pad cu valorile de la margine, altfel convolve pune zerouri la capete și razele de la ±fov par scurte
        r = np.convolve(np.pad(r, k // 2, mode='edge'), np.ones(k)/k, mode='valid')
        return r,a

    def extend_disparities(self, r, inc):
        out= r.copy()
        jumps=np.where(np.abs(np.diff(r)) > self.disparity_threshold)[0]
        for i in jumps:
            near = min(r[i], r[i+1]) 
            if near<= 0.0:
                continue
            n= int(math.ceil((self.car_half_width/near)/inc))
            if r[i+1]>r[i]:
                out[i+1:i+1+n]=np.minimum(out[i+1:i+1+n],near)
            else:
                lo=max(i-n+1,0)
                out[lo:i+1]=np.minimum(out[lo:i+1],near)
        return out
    def apply_bubble(self, r, inc):
        if not np.any(r > 0):
            return r
        i = int(np.argmin(np.where(r > 0, r, np.inf)))
        n = int(math.ceil((self.bubble_radius / r[i]) / inc))
        r[max(i - n, 0): i + n + 1] = 0.0
        return r
    def find_max_gap(self,free):
        # scorul unui gap = suma distanțelor (lățime × adâncime), nu doar numărul de raze
        best_start,best_end,best_score = 0, -1, 0.0
        start= None
        for i, val in enumerate(free):
            if val>0 and start is None:
                start=i
            if(val<=0 or i==len(free)-1) and start is not None:
                end=i if val>0 else i-1
                score = float(np.sum(free[start:end+1]))
                if score > best_score:
                    best_start, best_end, best_score = start, end, score
                start = None
        return best_start,best_end
    def find_best_point(self,start_i,end_i,ranges):
        gap = ranges[start_i:end_i+1]
        deep = gap >= self.deep_ratio * np.max(gap)
        # cea mai lată zonă adâncă (cu max_range multe raze sunt egale, argmax ar lua-o pe prima)
        best_lo, best_len = 0, 0
        lo = None
        for i, d in enumerate(deep):
            if d and lo is None:
                lo = i
            if (not d or i == len(deep) - 1) and lo is not None:
                hi = i if d else i - 1
                if hi - lo + 1 > best_len:
                    best_lo, best_len = lo, hi - lo + 1
                lo = None
        return start_i + best_lo + best_len // 2
    def lidar_callback(self, data):
        ranges = np.array(data.ranges)
        angles = data.angle_min + np.arange(len(ranges)) * data.angle_increment

        r, a = self.preprocess_lidar(ranges, angles)
        r = self.extend_disparities(r, data.angle_increment)
        r = self.apply_bubble(r, data.angle_increment)
        start_i, end_i = self.find_max_gap(r)

        drive = AckermannDriveStamped()
        if end_i < start_i:
            drive.drive.speed = 0.0
            self.drive_pub.publish(drive)
            return

        best_i = self.find_best_point(start_i, end_i, r)
        target = float(np.clip(a[best_i], -self.max_steer, self.max_steer))
        self.steer = self.steer_alpha * target + (1 - self.steer_alpha) * self.steer

        ratio = abs(self.steer) / self.max_steer
        speed = self.max_speed - (self.max_speed - self.min_speed) * ratio

        drive.drive.steering_angle = self.steer
        drive.drive.speed = speed
        self.drive_pub.publish(drive)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()