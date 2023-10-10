import rclpy
from rclpy.node import Node
from pynput import keyboard
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
import threading

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
        self.poses = []
        self.current_pose = None
        self.pose_id = 1

        self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        print('Press "s" to save the current pose, "q" to quit and save to csv.')

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.start()
        #with keyboard.Listener(on_press=self.on_key_press) as listener:
        #    listener.join()

    def pose_callback(self, msg):
        #print('subscribed!')
        self.current_pose = msg.pose.pose

    def keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_key_press) as listener:
            listener.join()

    def on_key_press(self, key):
        try:
            if key.char == 's':
                if self.current_pose is not None:
                    pose_data = [
                        str(self.pose_id),
                        str(self.current_pose.position.x),
                        str(self.current_pose.position.y),
                        str(self.current_pose.position.z),
                        str(self.current_pose.orientation.x),
                        str(self.current_pose.orientation.y),
                        str(self.current_pose.orientation.z),
                        str(self.current_pose.orientation.w)
                    ]
                    self.poses.append(pose_data)
                    print('Pose saved!')
                    self.pose_id += 1
                else:
                    print('Warning: No pose received yet.')
            elif key.char == 'q':
                self.save_poses_to_csv()
                self.destroy_node()
                rclpy.shutdown()
        except AttributeError:
            pass

    def save_poses_to_csv(self):
        filename = datetime.now().strftime('%Y%m%d%H%M%S') + '_waypoints.csv'
        path = os.path.join(os.path.curdir, 'waypoints', filename)  

        os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['id', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w'])
            writer.writerows(self.poses)

        print(f'Data saved to {path}!')

def main(args=None):
    rclpy.init(args=args)
    pose_recorder = PoseRecorder()

    try:
        rclpy.spin(pose_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

