import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# Nod de wall following (Lab 3): urmărește un perete cu un PID pe distanța
# prezisă până la perete. Citește doar /scan, publică pe /drive.
class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.drive_pub=self.create_publisher(AckermannDriveStamped,'/drive',10)

        # --- Geometrie ---
        self.desired_dist =0.9                 # [m] setpoint: distanța dorită față de perete
        self.theta =math.radians(50)           # unghiul dintre razele a și b (enunț: 0-70°)
        self.lookahead=1.0                     # [m] L: cât de departe în față prezicem distanța

        # --- Ce perete urmărim: 'left' (implicit, tur CCW) sau 'right' ---
        self.declare_parameter('follow_side', 'left')
        self.side = 1 if self.get_parameter('follow_side').value == 'left' else -1   # +1 stânga, -1 dreapta

        # --- PID ---
        self.kp=1.0
        self.ki=0.0
        self.kd=0.1
        self.max_steer = 0.4                   # [rad] limita fizică a direcției (~23°)
        self.max_integral=1.0                  # anti-windup: limita termenului integral
        self.d_alpha=0.1                       # filtru trece-jos pe D (0 = filtrare maximă, 1 = fără)

        # --- Detecția ușilor (o ușă arată ca un colț, dar în față e liber) ---
        self.doorway_dist=1.1 #de reglat       # [m] peste atât, peretele lateral a „dispărut”
        self.doorway_a = 1.7
        self.corner_front_dist=4.0 #de reglat  # [m] liber în față peste atât => ușă, nu colț

        # --- Memoria PID-ului ---
        self.integral= 0.0
        self.prev_error= 0.0
        self.prev_time=None                    # None = încă n-a venit niciun scan
        self.d_filt=0.0                        # derivata filtrată

    # Distanța măsurată la un unghi dat: media razelor valide din jurul lui.
    def get_range(self, msg, angle):
        # unghi -> indexul razei (inversul formulei din Lab 2)
        i=int(round((angle- msg.angle_min)/msg.angle_increment))
        # 5 raze în jurul lui i; media reduce zgomotul LiDAR-ului
        window =np.array(msg.ranges[max(i-2,0):i+3])
        window = window[np.isfinite(window)]   # scoate inf / nan
        if len(window) == 0:
            return msg.range_max               # nicio rază validă => „liber”
        return float(np.mean(window))

    # Eroarea față de setpoint, calculată pe distanța PREZISĂ până la perete.
    def get_error(self,msg,dist):
        # b perpendicular pe perete (±90°), a cu theta mai în față, front drept înainte
        b=self.get_range(msg,self.side * math.pi/2)
        a=self.get_range(msg,self.side * (math.pi/2-self.theta))
        front=self.get_range(msg,0.0)
        self.get_logger().info(f'a={a:.2f} b={b:.2f} front={front:.2f}', throttle_duration_sec=0.2)

        # Ușă: peretele lateral dispare, dar în față e liber => păstrăm eroarea veche
        door = front > self.corner_front_dist
        a_bad = door and a > self.doorway_a
        b_bad = door and b > self.doorway_dist
        if a_bad and b_bad:
            return 0.0                                          # ambele în nișă: mergi drept
        if a_bad:
            return self.side * (b - dist)                       # distanța doar din b
        if b_bad:
            return self.side * (a * math.cos(self.theta) - dist)  # distanța doar din a

        # Geometria din enunț: alpha = unghiul mașinii față de perete (>0 = se îndepărtează)
        alpha=math.atan2(a*math.cos(self.theta) - b, a * math.sin(self.theta))
        d_now=b*math.cos(alpha)                               # D_t: distanța acum
        d_future=d_now + self.lookahead * math.sin(alpha)     # D_t+1: distanța după L metri
        # side face ca eroarea pozitivă să însemne mereu „virează la stânga”
        return self.side * (d_future - dist)

    # PID discret: întoarce unghiul de viraj [rad], limitat la ±max_steer.
    def pid_control(self,error,t):
        # primul scan: nu avem încă derivată, doar salvăm starea și mergem drept
        if self.prev_time is None:
            self.prev_time = t
            self.prev_error =error
            return 0.0
        dt=max(t-self.prev_time, 1e-3)         # evită împărțirea la 0

        # I: integrare dreptunghi + anti-windup
        self.integral +=  error * dt
        self.integral= float(np.clip(self.integral, -self.max_integral, self.max_integral))

        # D: diferență finită + filtru trece-jos (scan la 250 Hz => derivata brută e zgomotoasă)
        raw_d = (error -self.prev_error) / dt
        self.d_filt = self.d_alpha * raw_d + (1- self.d_alpha) * self.d_filt
        self.prev_error = error
        self.prev_time = t

        # u = Kp*e + Ki*∫e + Kd*de/dt, saturat la limita direcției
        angle= self.kp* error + self.ki * self.integral +self.kd* self.d_filt
        return float(np.clip(angle,-self.max_steer,self.max_steer))

    # La fiecare scan: eroare -> PID -> viteză după unghi -> publică pe /drive.
    def scan_callback(self,msg):
        t=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9     # momentul scan-ului [s]
        error = self.get_error(msg,self.desired_dist)
        angle =self.pid_control(error,t)

        # viteza în trepte după cât de tare virăm (regulile din enunț)
        steer_deg = abs(math.degrees(angle))
        if steer_deg<10:
            velocity =1.5
        elif steer_deg<20:
            velocity =1.0
        else:
            velocity =0.5

        drive = AckermannDriveStamped()
        drive.drive.steering_angle= angle
        drive.drive.speed=velocity
        self.drive_pub.publish(drive)


def main(args = None):
    rclpy.init(args=args)
    node = WallFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
