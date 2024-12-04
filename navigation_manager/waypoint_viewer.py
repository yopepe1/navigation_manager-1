import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import csv
import sys

class WaypointViewer(Node):
    def __init__(self, csv_path):
        super().__init__('waypoint_viewer')
        self.markers = MarkerArray()
        self.text_markers = MarkerArray()
        self.line_markers = MarkerArray()

        # MarkerArrayパブリッシャーを作成
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10)
        self.text_marker_pub = self.create_publisher(MarkerArray, 'waypoint/text_markers', 10)
        self.line_marker_pub = self.create_publisher(MarkerArray, 'waypoint/line_markers', 10)
        self.waypoint_id_sub = self.create_subscription(Int32, 'next_waypoint_id', self.waypoint_id_callback, 10)

        # CSVファイルからウェイポイントを読み込む
        self.waypoints = self.load_waypoints_from_csv(csv_path)
        
        # マーカーを定期的に公開するためのタイマー
        self.timer = self.create_timer(1.0, self.publish_markers)
        
    def load_waypoints_from_csv(self, filename):
        waypoints = []
        try:
            with open(filename, mode='r') as file:
                reader = csv.reader(file)
                next(reader)  # ヘッダーをスキップ
                for row in reader:
                    waypoint_data = {
                        'id': int(row[0]),
                        'pose': PoseStamped()
                    }
                    waypoint_data['pose'].header.frame_id = 'map'
                    waypoint_data['pose'].pose.position.x = float(row[1])
                    waypoint_data['pose'].pose.position.y = float(row[2])
                    waypoint_data['pose'].pose.position.z = float(row[3])
                    waypoint_data['pose'].pose.orientation.x = float(row[4])
                    waypoint_data['pose'].pose.orientation.y = float(row[5])
                    waypoint_data['pose'].pose.orientation.z = float(row[6])
                    waypoint_data['pose'].pose.orientation.w = float(row[7])
                    waypoints.append(waypoint_data)
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
        return waypoints

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # ウェイポイントを示すマーカー
        for i, waypoint in enumerate(self.waypoints):
            # 矢印マーカー
            arrow_marker = Marker()
            arrow_marker.header.frame_id = 'map'
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = 'waypoint_arrows'
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose = waypoint['pose'].pose
            arrow_marker.scale.x = 0.5  # 矢印の長さ
            arrow_marker.scale.y = 0.1  # 矢印の幅
            arrow_marker.scale.z = 0.1  # 矢印の高さ
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.0
            arrow_marker.color.b = 1.0
            arrow_marker.color.a = 1.0
            self.line_markers.markers.append(arrow_marker)
            
            # ウェイポイントIDを表示するテキストマーカー
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_numbers'
            text_marker.id = i + 1000  # IDの重複を避けるため
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = waypoint['pose'].pose
            text_marker.pose.position.z += 0.5  # テキストを少し上に表示
            text_marker.text = str(waypoint['id'])  # CSVファイルの元のID
            text_marker.scale.z = 0.3  # テキストのサイズ
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            self.text_markers.markers.append(text_marker)

    def timer_callback(self):
        self.marker_pub.publish(self.markers)
        self.text_marker_pub.publish(self.text_markers)
        self.line_marker_pub.publish(self.line_markers)


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run navigation_manager waypoint_viewer <path_to_csv>")
        rclpy.shutdown()
        return
        
    csv_path = sys.argv[1].replace('waypoints_csv:=', '')
    
    try:
        waypoint_viewer = WaypointViewer(csv_path)
        rclpy.spin(waypoint_viewer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'waypoint_viewer' in locals():
            waypoint_viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
