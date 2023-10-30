import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import csv
import time
from std_msgs.msg import Int32

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        self.declare_parameter('filename', 'waypoints.csv')
        self.declare_parameter('action_server_name', 'navigate_to_pose')
        waypoints_filename = self.get_parameter('filename').value
        action_server_name = self.get_parameter('action_server_name').value
#        print(waypoints_filename)

        self.publisher_ = self.create_publisher(Int32, '/navigation_manager/next_waypointID', 10)

        self._action_client = ActionClient(self, NavigateToPose, action_server_name)
        self.waypoints = self.load_waypoints_from_csv(waypoints_filename)
        self.current_waypoint_index = 0
        self._last_feedback_time = self.get_clock().now()
        self.xy_goal_tol_ = 2.0
        self.des_lin_vel_ = 0.4
        self.stop_flag_ = 0
        self.skip_flag_ = 1

    def load_waypoints_from_csv(self, filename):
        waypoints = []
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                pose_stamped_msg = PoseStamped()
                pose_stamped_msg.header.frame_id = 'map'  # Adjust the frame_id as needed
                pose_stamped_msg.pose.position.x = float(row[1])
                pose_stamped_msg.pose.position.y = float(row[2])
                pose_stamped_msg.pose.position.z = float(row[3])
                pose_stamped_msg.pose.orientation.x = float(row[4])
                pose_stamped_msg.pose.orientation.y = float(row[5])
                pose_stamped_msg.pose.orientation.z = float(row[6])
                pose_stamped_msg.pose.orientation.w = float(row[7])
                waypoints.append(pose_stamped_msg)
                self.xy_goal_tol_ = float(row[8])
                self.des_lin_vel_ = float(row[9])
                self.stop_flag_ = bool(row[10])
                self.skip_flag_ = bool(row[11])

#                print(self.xy_goal_tol_)
#                print(self.des_lin_vel_)
#                print(self.stop_flag_)
#                print(self.skip_flag_)

        return waypoints

    def send_goal(self, pose_stamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available, waiting...')
        
        self.get_logger().info('Sending waypoint...')
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        int_msg = Int32(data=self.current_waypoint_index)
        self.publisher_.publish(int_msg)

    def feedback_callback(self, feedback_msg):
        current_time = self.get_clock().now()
        if (current_time - self._last_feedback_time).nanoseconds >= 3e9:
            self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.distance_remaining))
            self._last_feedback_time = current_time

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            # Resend the goal or handle accordingly
            # ...

        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal completed with result: {0}'.format(result))
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index < len(self.waypoints):
            next_waypoint = self.waypoints[self.current_waypoint_index]
            next_waypoint.header.stamp = self.get_clock().now().to_msg()
            self.send_goal(next_waypoint)

    def run(self):
        if self.waypoints:
            self.send_goal(self.waypoints[self.current_waypoint_index])

def main(args=None):
    rclpy.init(args=args)

    waypoint_sender = WaypointSender()
    try:
        waypoint_sender.run()
        rclpy.spin(waypoint_sender)
    except KeyboardInterrupt:
        print("Received KeyboardInterrupt, shutting down...")
    finally:
        waypoint_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

