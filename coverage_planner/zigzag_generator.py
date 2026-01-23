import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, PoseStamped, Point32
from nav_msgs.msg import Path
import math

class ZigzagGenerator(Node):
    def __init__(self):
        super().__init__('zigzag_generator')
        
        # パラメータの宣言 (ロボットの幅 = ジグザグの間隔)
        self.declare_parameter('robot_width', 0.5)  # 0.3 -> 0.5 (密度緩和)
        # マージンパラメータ (ロボットの半径 + 安全余裕)
        # インフレーション半径(0.3m) + 対角線半径(0.354m) + オーバーシュート(0.1m) = 0.754m
        self.declare_parameter('safety_margin', 0.75) # 0.45 -> 0.75 

        # /cone_area トピックをサブスクライブ
        self.subscription = self.create_subscription(
            PolygonStamped,
            '/cone_area',
            self.area_callback,
            10)
        
        # /coverage_path トピックに Path をパブリッシュ
        self.path_publisher_ = self.create_publisher(Path, '/coverage_path', 10)
        
        self.get_logger().info('Zigzag Generator started. Waiting for /cone_area...')

    def area_callback(self, msg: PolygonStamped):
        """
        多角形エリアを受信し、ジグザグ経路を生成するコールバック関数
        最長辺に沿った方向でジグザグを生成する
        """
        self.get_logger().info('Received polygon area. Generating path...')

        robot_width = self.get_parameter('robot_width').get_parameter_value().double_value
        margin = self.get_parameter('safety_margin').get_parameter_value().double_value

        if robot_width <= 0:
            self.get_logger().error('robot_width must be greater than 0')
            return

        points = msg.polygon.points
        if len(points) < 3:
            self.get_logger().warn('Polygon has less than 3 points.')
            return

        # 1. ポリゴンを縮小 (Offset Polygon)
        inner_polygon = self.offset_polygon(points, margin)

        if not inner_polygon or len(inner_polygon) < 3:
            self.get_logger().warn('Inner polygon is invalid or too small (margin might be too large).')
            return

        # 2. 最長辺を見つけて、その方向を基準軸とする
        sweep_angle = self.find_longest_edge_angle(inner_polygon)
        self.get_logger().info(f'Sweep angle (longest edge): {math.degrees(sweep_angle):.1f} degrees')

        # 3. ポリゴンを回転して軸に揃える
        rotated_polygon = self.rotate_polygon(inner_polygon, -sweep_angle)

        # 4. 回転後のポリゴンの範囲 (AABB) を計算
        min_x = min(p.x for p in rotated_polygon)
        max_x = max(p.x for p in rotated_polygon)

        # 5. ジグザグ経路 (Path) のウェイポイントを生成
        path_msg = Path()
        path_msg.header = msg.header

        # アプローチ用ウェイポイントの追加
        first_intersections = self.get_line_intersections(rotated_polygon, min_x)
        if len(first_intersections) >= 2:
            first_intersections.sort()
            first_y = first_intersections[0]
            first_x = min_x

            centroid_x, centroid_y = self.calculate_centroid(rotated_polygon)

            dx = centroid_x - first_x
            dy = centroid_y - first_y
            dist_to_centroid = math.hypot(dx, dy)

            approach_dist = 1.0
            if dist_to_centroid < approach_dist:
                approach_dist = dist_to_centroid * 0.8

            if dist_to_centroid > 1e-6:
                ax = first_x + (dx / dist_to_centroid) * approach_dist
                ay = first_y + (dy / dist_to_centroid) * approach_dist

                # 回転座標系から元の座標系に戻す
                ax_orig, ay_orig = self.rotate_point(ax, ay, sweep_angle)
                fx_orig, fy_orig = self.rotate_point(first_x, first_y, sweep_angle)

                dir_x = fx_orig - ax_orig
                dir_y = fy_orig - ay_orig

                path_msg.poses.append(self.create_pose(ax_orig, ay_orig, msg.header, dir_x, dir_y))
                self.get_logger().info(f'Added approach waypoint at ({ax_orig:.2f}, {ay_orig:.2f})')

        current_x = min_x
        direction = 1

        while current_x <= max_x:
            intersections = self.get_line_intersections(rotated_polygon, current_x)

            if len(intersections) >= 2:
                intersections.sort()

                if len(intersections) % 2 != 0:
                    pass
                else:
                    segments = []
                    for i in range(0, len(intersections), 2):
                        y_start = intersections[i]
                        y_end = intersections[i+1]
                        segments.append((y_start, y_end))

                    if direction == 1:
                        for y_s, y_e in segments:
                            # 回転座標系から元の座標系に戻してから追加
                            self.add_interpolated_poses_rotated(
                                path_msg, current_x, y_s, current_x, y_e,
                                msg.header, sweep_angle)
                    else:
                        for y_s, y_e in reversed(segments):
                            self.add_interpolated_poses_rotated(
                                path_msg, current_x, y_e, current_x, y_s,
                                msg.header, sweep_angle)

            current_x += robot_width
            direction *= -1

        self.path_publisher_.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} waypoints.')

    def find_longest_edge_angle(self, polygon):
        """
        ポリゴンの最長辺の角度を返す
        """
        max_length = 0
        angle = 0
        num_points = len(polygon)

        for i in range(num_points):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % num_points]

            dx = p2.x - p1.x
            dy = p2.y - p1.y
            length = math.hypot(dx, dy)

            if length > max_length:
                max_length = length
                angle = math.atan2(dy, dx)

        return angle

    def rotate_point(self, x, y, angle):
        """
        点を原点周りに回転
        """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        new_x = x * cos_a - y * sin_a
        new_y = x * sin_a + y * cos_a
        return new_x, new_y

    def rotate_polygon(self, polygon, angle):
        """
        ポリゴン全体を原点周りに回転
        """
        rotated = []
        for p in polygon:
            new_x, new_y = self.rotate_point(p.x, p.y, angle)
            p_new = Point32()
            p_new.x = float(new_x)
            p_new.y = float(new_y)
            p_new.z = 0.0
            rotated.append(p_new)
        return rotated

    def add_interpolated_poses_rotated(self, path_msg, x1, y1, x2, y2, header, angle, resolution=0.3):
        """
        回転座標系の2点間を補間し、元の座標系に戻してpath_msgに追加
        """
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6:
            ox, oy = self.rotate_point(x1, y1, angle)
            path_msg.poses.append(self.create_pose(ox, oy, header, 1.0, 0.0))
            return

        num_points = int(dist / resolution)
        if num_points < 1:
            num_points = 1

        for i in range(num_points + 1):
            t = i / float(num_points)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            # 元の座標系に戻す
            ox, oy = self.rotate_point(x, y, angle)

            # 次の点との方向を計算（最後の点は前の方向を維持）
            if i < num_points:
                next_x = x1 + (i + 1) / float(num_points) * (x2 - x1)
                next_y = y1 + (i + 1) / float(num_points) * (y2 - y1)
                next_ox, next_oy = self.rotate_point(next_x, next_y, angle)
                dir_x = next_ox - ox
                dir_y = next_oy - oy
            else:
                # 最後の点は前の方向を使う
                prev_x = x1 + (num_points - 1) / float(num_points) * (x2 - x1)
                prev_y = y1 + (num_points - 1) / float(num_points) * (y2 - y1)
                prev_ox, prev_oy = self.rotate_point(prev_x, prev_y, angle)
                dir_x = ox - prev_ox
                dir_y = oy - prev_oy

            path_msg.poses.append(self.create_pose(ox, oy, header, dir_x, dir_y))

    def add_interpolated_poses(self, path_msg, x1, y1, x2, y2, header, resolution=0.3):
        """
        2点間 (x1, y1) -> (x2, y2) を resolution 間隔で補間し、path_msg に追加する
        """
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6:
            # 距離がほぼ0なら始点のみ追加（またはスキップ）
            path_msg.poses.append(self.create_pose(x1, y1, header, 1.0, 0.0))
            return
        
        # Calculate direction vector for orientation
        direction_x = (x2 - x1) / dist
        direction_y = (y2 - y1) / dist

        num_points = int(dist / resolution)
        if num_points < 1:
            num_points = 1
        
        for i in range(num_points + 1):
            t = i / float(num_points)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            path_msg.poses.append(self.create_pose(x, y, header, direction_x, direction_y))

    def create_pose(self, x, y, header, direction_x=1.0, direction_y=0.0):
        """
        Create a PoseStamped with proper orientation based on direction vector
        """
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Calculate yaw from direction vector
        yaw = math.atan2(direction_y, direction_x)
        
        # Convert yaw to quaternion
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
        
        # CCW (反時計回り) かどうか
        is_ccw = area > 0
        
        num_points = len(points)
        new_points = []
        
        # 各頂点について、隣接する2つのエッジを内側にシフトした直線の交点を求める
        for i in range(num_points):
            p_prev = points[(i - 1) % num_points]
            p_curr = points[i]
            p_next = points[(i + 1) % num_points]
            
            # エッジベクトル
            v1x, v1y = p_curr.x - p_prev.x, p_curr.y - p_prev.y
            v2x, v2y = p_next.x - p_curr.x, p_next.y - p_curr.y
            
            l1 = math.hypot(v1x, v1y)
            l2 = math.hypot(v2x, v2y)
            
            if l1 < 1e-6 or l2 < 1e-6:
                continue
                
            # 正規化
            v1x /= l1
            v1y /= l1
            v2x /= l2
            v2y /= l2
            
            # 内向き法線ベクトル
            if is_ccw:
                n1x, n1y = -v1y, v1x
                n2x, n2y = -v2y, v2x
            else:
                n1x, n1y = v1y, -v1x
                n2x, n2y = v2y, -v2x
                
            # シフトした直線の交点を求める
            # 直線1: n1x * x + n1y * y = d1
            # d1 = n1x * (p_curr.x + n1x*margin) + n1y * (p_curr.y + n1y*margin)
            d1 = n1x * p_curr.x + n1y * p_curr.y + margin
            
            # 直線2: n2x * x + n2y * y = d2
            d2 = n2x * p_curr.x + n2y * p_curr.y + margin
            
            # 連立方程式を解く (クラメルの公式)
            # | n1x n1y | | x | = | d1 |
            # | n2x n2y | | y |   | d2 |
            
            det = n1x * n2y - n1y * n2x
            
            if abs(det) < 1e-6:
                # 平行または直線状。頂点はそのまま法線方向にシフト
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

    def get_line_intersections(self, polygon, x):
        """
        垂直線 X = x とポリゴンのエッジの交点 (Y座標) のリストを返す
        """
        intersections = []
        num = len(polygon)
        for i in range(num):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % num]
            
            # X座標の範囲チェック (線分が垂直線と交差するか)
            # 端点を含むかどうかの判定に注意 (重複カウントを防ぐため < を使用)
            if (p1.x <= x < p2.x) or (p2.x <= x < p1.x):
                # 線形補間
                t = (x - p1.x) / (p2.x - p1.x)
                y = p1.y + t * (p2.y - p1.y)
                intersections.append(y)
                
        return intersections

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
    node = ZigzagGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
