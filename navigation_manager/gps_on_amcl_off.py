import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class GpsOnAmclOff(Node):
    def __init__(self):
        super().__init__('gps_on_amcl_off')
        self.gps_pose_enable_publisher_ = self.create_publisher(Int32, '/navigation_manager/gps_pose_enable', 10)
        self.amcl_pose_enable_publisher_ = self.create_publisher(Int32, '/navigation_manager/amcl_pose_enable', 10)

    def publish_messages(self):
        for _ in range(3):
            self.gps_pose_enable_publisher_.publish(Int32(data=1))
            self.amcl_pose_enable_publisher_.publish(Int32(data=0))
            time.sleep(1)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GpsOnAmclOff()
    node.publish_messages()

if __name__ == '__main__':
    main()

