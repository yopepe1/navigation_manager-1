import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import csv
import time
from std_msgs.msg import Int32
import threading
from pynput import keyboard

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
        self.waypoints_data = self.load_waypoints_from_csv(waypoints_filename)
        self.current_waypoint_index = 0
        self._last_feedback_time = self.get_clock().now()
        self.xy_goal_tol_ = 2.0
        self.des_lin_vel_ = 0.4
        self.stop_flag_ = 0
        self.skip_flag_ = 1

    def load_waypoints_from_csv(self, filename):
        waypoints_data = []
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

                waypoint_data = {
                    "pose": pose_stamped_msg,
                    "xy_goal_tol": float(row[8]),
                    "des_lin_vel": float(row[9]),
                    "stop_flag": int(row[10]),
                    "skip_flag": int(row[11])
                }

                #self.get_logger().info('stop_flag: %s' % waypoint_data["stop_flag"])
                #self.get_logger().info('stop_flag: %d' % waypoint_data["stop_flag"])
                #self.get_logger().info('skip_flag: %s' % waypoint_data["skip_flag"])

                waypoints_data.append(waypoint_data)

#                print(self.xy_goal_tol_)
#                print(self.des_lin_vel_)
#                print(self.stop_flag_)
#                print(self.skip_flag_)

        return waypoints_data

    def send_goal(self, waypoint_data):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint_data["pose"]
        
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available, waiting...')
        
        self.get_logger().info('Sending waypoint...')
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        int_msg = Int32(data=self.current_waypoint_index)
        self.publisher_.publish(int_msg)
        # ここにros2 paramを送信するものを書く

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
        status = future.result().status
        self.get_logger().info('Goal completed with result: {0}'.format(result))

        self.next_waypoint_data = self.waypoints_data[self.current_waypoint_index]
        current_stop_flag = self.next_waypoint_data["stop_flag"]
        current_skip_flag = self.next_waypoint_data["skip_flag"]
        #self.get_logger().info('stop_flag: %s' % current_stop_flag)
        self.get_logger().info('skip_flag: %s' % current_skip_flag)
        self.get_logger().info('status: %s' % status)
        
        # SUCCEEDED または skip == 1のときに次のwaypointを目指す
        # ABORTED かつ skip == 0 のときだけ同じwaypointを目指す
        if current_skip_flag == 1 or status == GoalStatus.STATUS_SUCCEEDED:
            self.current_waypoint_index += 1
        #self.get_logger().info('waypoint_index: %d' % self.current_waypoint_index)
        
        if self.current_waypoint_index < len(self.waypoints_data):
            self.next_waypoint_data = self.waypoints_data[self.current_waypoint_index]

            if current_stop_flag == 1:
                self.get_logger().info('Press n key to resume navigation.')
                threading.Thread(target=self.wait_for_user_input).start()
            else:
                self.next_waypoint_data["pose"].header.stamp = self.get_clock().now().to_msg()
                self.send_goal(self.next_waypoint_data)

    def run(self):
        if self.waypoints_data:
            self.send_goal(self.waypoints_data[self.current_waypoint_index])

    def wait_for_user_input(self):
        self.key_pressed = False

        def on_press(key):
            if key == keyboard.KeyCode.from_char('n'):
                self.key_pressed = True
                return False  # Stop the listener

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

        print("'n' key detected!")
        self.next_waypoint_data["pose"].header.stamp = self.get_clock().now().to_msg()
        #self.send_goal(self.next_waypoint_data["pose"])
        self.send_goal(self.next_waypoint_data)

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

