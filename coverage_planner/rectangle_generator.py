import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, PoseStamped, Point32
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math

class RectangleGenerator(Node):
    def __init__(self):
        super().__init__('rectangle_generator')

        # パラメータの宣言
        # マージンパラメータ (ロボットの半径 + 安全余裕)
        # インフレーション半径(0.3m) + 対角線半径(0.354m) + オーバーシュート(0.1m) = 0.754m
        self.declare_parameter('safety_margin', 0.75)

        # TF2 setup for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /cone_area トピックをサブスクライブ
        self.subscription = self.create_subscription(
            PolygonStamped,
            '/cone_area',
            self.area_callback,
            10)

        # /coverage_path トピックに Path をパブリッシュ
        self.path_publisher_ = self.create_publisher(Path, '/coverage_path', 10)

        self.get_logger().info('Rectangle Generator started. Waiting for /cone_area...')

    def area_callback(self, msg: PolygonStamped):
        """
        多角形エリアを受信し、外周経路を生成するコールバック関数
        """
        self.get_logger().info('Received polygon area. Generating rectangle path...')
        
        margin = self.get_parameter('safety_margin').get_parameter_value().double_value
        
        points = msg.polygon.points
        if len(points) < 3:
            self.get_logger().warn('Polygon has less than 3 points.')
            return

        # 1. ポリゴンを縮小 (Offset Polygon)
        inner_polygon = self.offset_polygon(points, margin)
        
        if not inner_polygon or len(inner_polygon) < 3:
            self.get_logger().warn('Inner polygon is invalid or too small (margin might be too large).')
            return

        # 2. 経路 (Path) のウェイポイントを生成
        path_msg = Path()
        path_msg.header = msg.header

        # ロボットの現在位置と向きを取得
        try:
            transform = self.tf_buffer.lookup_transform(
                msg.header.frame_id,  # target frame (usually 'map' or 'odom')
                'base_link',          # source frame
                rclpy.time.Time(),    # latest available
                rclpy.duration.Duration(seconds=1.0))

            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            # quaternion to yaw
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            robot_yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))

            robot_dir_x = math.cos(robot_yaw)
            robot_dir_y = math.sin(robot_yaw)

            self.get_logger().info(f'Robot pose: ({robot_x:.2f}, {robot_y:.2f}), yaw: {robot_yaw:.2f} rad')

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get robot pose: {e}. Falling back to longest edge.')
            # Fallback: 最長辺を使う
            robot_x, robot_y = 0.0, 0.0
            robot_dir_x, robot_dir_y = 1.0, 0.0

        # ロボットの前方にある辺を見つける
        num_points = len(inner_polygon)
        max_dot_product = -999.0
        forward_edge_index = 0

        for i in range(num_points):
            p1 = inner_polygon[i]
            p2 = inner_polygon[(i + 1) % num_points]

            # 辺の中点
            mid_x = (p1.x + p2.x) / 2.0
            mid_y = (p1.y + p2.y) / 2.0

            # ロボットから辺の中点へのベクトル
            to_mid_x = mid_x - robot_x
            to_mid_y = mid_y - robot_y

            # 正規化
            to_mid_len = math.hypot(to_mid_x, to_mid_y)
            if to_mid_len < 1e-6:
                continue
            to_mid_x /= to_mid_len
            to_mid_y /= to_mid_len

            # ロボットの前方方向との内積
            dot = robot_dir_x * to_mid_x + robot_dir_y * to_mid_y

            if dot > max_dot_product:
                max_dot_product = dot
                forward_edge_index = i

        # 前方の辺の始点と終点
        p_start_edge = inner_polygon[forward_edge_index]
        p_end_edge = inner_polygon[(forward_edge_index + 1) % num_points]

        # 辺の長さと方向ベクトル
        edge_len = math.hypot(p_end_edge.x - p_start_edge.x, p_end_edge.y - p_start_edge.y)
        if edge_len < 1e-6:
             self.get_logger().warn('Edge length is too small.')
             return

        dir_x = (p_end_edge.x - p_start_edge.x) / edge_len
        dir_y = (p_end_edge.y - p_start_edge.y) / edge_len

        # 前方の辺の真ん中 (Start Point)
        start_pt_x = p_start_edge.x + dir_x * (edge_len * 0.5)
        start_pt_y = p_start_edge.y + dir_y * (edge_len * 0.5)
        
        # アプローチポイント (Start Point から逆方向に approach_dist 戻る)
        # ただし、辺の始点 (p_start_edge) を超えないように制限する
        # Start Point は端から1/4の位置なので、p_start_edge までの距離は edge_len * 0.25

        dist_to_corner = edge_len * 0.25
        approach_dist = 1.0 # 希望するアプローチ距離

        # 安全のため、コーナーより少し内側までしか戻らないようにする (マージン 0.2m)
        max_approach_dist = max(0.0, dist_to_corner - 0.2)

        actual_approach_dist = min(approach_dist, max_approach_dist)
        
        approach_pt_x = start_pt_x - dir_x * actual_approach_dist
        approach_pt_y = start_pt_y - dir_y * actual_approach_dist
        
        # 1. アプローチポイント -> スタートポイント (直線)
        self.add_interpolated_poses(path_msg, approach_pt_x, approach_pt_y, start_pt_x, start_pt_y, msg.header)
        self.get_logger().info(f'Added approach path from ({approach_pt_x:.2f}, {approach_pt_y:.2f}) to ({start_pt_x:.2f}, {start_pt_y:.2f})')
        
        # 2. スタートポイント -> 最長辺の終点
        self.add_interpolated_poses(path_msg, start_pt_x, start_pt_y, p_end_edge.x, p_end_edge.y, msg.header)
        
        # 3. 残りの辺を一周
        # forward_edge_index + 1 から順に回る
        current_idx = (forward_edge_index + 1) % num_points
        
        for i in range(num_points - 1): # 最後の辺（戻ってくる辺）以外
            p_curr = inner_polygon[current_idx]
            p_next = inner_polygon[(current_idx + 1) % num_points]
            
            self.add_interpolated_poses(path_msg, p_curr.x, p_curr.y, p_next.x, p_next.y, msg.header)
            current_idx = (current_idx + 1) % num_points
            
        # 4. 最後の辺の始点 -> スタートポイント (閉じる)
        # current_idx は今 forward_edge_index になっているはず
        p_last = inner_polygon[current_idx] # = p_start_edge
        self.add_interpolated_poses(path_msg, p_last.x, p_last.y, start_pt_x, start_pt_y, msg.header)

        self.path_publisher_.publish(path_msg)
        self.get_logger().info(f'Published rectangle path with {len(path_msg.poses)} waypoints.')

    def add_interpolated_poses(self, path_msg, x1, y1, x2, y2, header, resolution=0.1):
        """
        2点間 (x1, y1) -> (x2, y2) を resolution 間隔で補間し、path_msg に追加する
        """
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6:
            path_msg.poses.append(self.create_pose(x1, y1, header, 1.0, 0.0))
            return
        
        direction_x = (x2 - x1) / dist
        direction_y = (y2 - y1) / dist

        num_points = int(dist / resolution)
        if num_points < 1:
            num_points = 1
        
        # 始点を含み、終点は次のセグメントの始点となるため、ループの最後で追加される
        # ただし、最後のセグメントの終点は始点に戻るため、閉じる必要がある
        # ここでは単純に各セグメントの始点〜終点直前までを追加し、最後に全体を閉じるか、
        # あるいは重複を許容して終点まで追加するか。
        # zigzag_generatorの実装に合わせて、始点から終点まで追加する形にする。
        
        for i in range(num_points + 1):
            t = i / float(num_points)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            path_msg.poses.append(self.create_pose(x, y, header, direction_x, direction_y))

    def create_pose(self, x, y, header, direction_x=1.0, direction_y=0.0):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        yaw = math.atan2(direction_y, direction_x)
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def calculate_polygon_area(self, points):
        area = 0.0
        for i in range(len(points)):
            j = (i + 1) % len(points)
            area += points[i].x * points[j].y
            area -= points[j].x * points[i].y
        return area / 2.0

    def offset_polygon(self, points, margin):
        """
        多角形を内側に margin だけ縮小した新しい頂点リストを返す
        """
        area = self.calculate_polygon_area(points)
        if abs(area) < 1e-6:
            return []
        
        is_ccw = area > 0
        
        num_points = len(points)
        new_points = []
        
        for i in range(num_points):
            p_prev = points[(i - 1) % num_points]
            p_curr = points[i]
            p_next = points[(i + 1) % num_points]
            
            v1x, v1y = p_curr.x - p_prev.x, p_curr.y - p_prev.y
            v2x, v2y = p_next.x - p_curr.x, p_next.y - p_curr.y
            
            l1 = math.hypot(v1x, v1y)
            l2 = math.hypot(v2x, v2y)
            
            if l1 < 1e-6 or l2 < 1e-6:
                continue
                
            v1x /= l1
            v1y /= l1
            v2x /= l2
            v2y /= l2
            
            if is_ccw:
                n1x, n1y = -v1y, v1x
                n2x, n2y = -v2y, v2x
            else:
                n1x, n1y = v1y, -v1x
                n2x, n2y = v2y, -v2x
                
            d1 = n1x * p_curr.x + n1y * p_curr.y + margin
            d2 = n2x * p_curr.x + n2y * p_curr.y + margin
            
            det = n1x * n2y - n1y * n2x
            
            if abs(det) < 1e-6:
                new_x = p_curr.x + n1x * margin
                new_y = p_curr.y + n1y * margin
            else:
                new_x = (d1 * n2y - d2 * n1y) / det
                new_y = (n1x * d2 - n2x * d1) / det
                
            p_new = Point32()
            p_new.x = float(new_x)
            p_new.y = float(new_y)
            p_new.z = 0.0
            new_points.append(p_new)
            
        return new_points

    def calculate_centroid(self, points):
        x = 0.0
        y = 0.0
        num = len(points)
        if num == 0:
            return 0.0, 0.0
            
        for p in points:
            x += p.x
            y += p.y
        return x / num, y / num

def main(args=None):
    rclpy.init(args=args)
    node = RectangleGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
