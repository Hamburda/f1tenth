import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        self.declare_parameter('v',0.0)
        self.declare_parameter('d',0.0)

        self.publisher = self.create_publisher(AckermannDriveStamped,'drive',10)

        self.timer =self.create_timer(0.0,self.timer_callback)

    def timer_callback(self):
        v=self.get_parameter('v').value
        d=self.get_parameter('d').value

        msg = AckermannDriveStamped()
        msg.drive.speed = float(v)
        msg.drive.steering_angle = float(d)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node =Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

