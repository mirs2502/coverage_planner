#!/usr/bin/env python3
"""
2つのコーン間を往復するパスジェネレータ

動作:
1. /accumulated_cones から最初の2つのコーンを取得
2. 各コーンから0.3m手前の点を計算
3. A -> B -> A -> B の往復パスを生成（無限ループ）
4. /coverage_path に Path メッセージとしてパブリッシュ
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import math


class TwoConeReciprocate(Node):
    def __init__(self):
        super().__init__('two_cone_reciprocate')

        # パラメータの宣言
        self.declare_parameter('stop_distance', 0.3)  # コーンから0.3m手前で停止
        self.declare_parameter('path_resolution', 0.3)  # 経路の解像度
        self.declare_parameter('num_cycles', 3)  # 往復回数（3往復 = A->B->A->B->A->B->A）

        # /accumulated_cones トピックをサブスクライブ
        self.subscription = self.create_subscription(
            PointCloud2,
            '/accumulated_cones',
            self.cones_callback,
            10)

        # /coverage_path トピックに Path をパブリッシュ
        self.path_publisher = self.create_publisher(Path, '/coverage_path', 10)

        self.path_generated = False

        self.get_logger().info('Two Cone Reciprocate started. Waiting for /accumulated_cones...')

    def cones_callback(self, msg: PointCloud2):
        """
        蓄積されたコーンを受信し、2つのコーン間の往復経路を生成
        """
        # 既にパス生成済みなら何もしない
        if self.path_generated:
            return

        # PointCloud2をパース
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append(point)

        if len(points) < 2:
            self.get_logger().warn(f'Need at least 2 cones, got {len(points)}')
            return

        self.get_logger().info(f'Received {len(points)} cones. Generating reciprocating path...')

        # 最初の2つのコーンを使用
        cone_a = points[0]
        cone_b = points[1]

        self.get_logger().info(f'Cone A: ({cone_a[0]:.2f}, {cone_a[1]:.2f})')
        self.get_logger().info(f'Cone B: ({cone_b[0]:.2f}, {cone_b[1]:.2f})')

        # パラメータ取得
        stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        resolution = self.get_parameter('path_resolution').get_parameter_value().double_value
        num_cycles = self.get_parameter('num_cycles').get_parameter_value().integer_value

        # 2つのコーン間のベクトルを計算
        dx = cone_b[0] - cone_a[0]
        dy = cone_b[1] - cone_a[1]
        distance = math.hypot(dx, dy)

        if distance < 2 * stop_distance:
            self.get_logger().error(f'Cones too close ({distance:.2f}m). Need at least {2*stop_distance:.2f}m.')
            return

        # 正規化された方向ベクトル
        dir_x = dx / distance
        dir_y = dy / distance

        # コーンAから0.3m手前の点（スタート地点）
        point_a_x = cone_a[0] + dir_x * stop_distance
        point_a_y = cone_a[1] + dir_y * stop_distance

        # コーンBから0.3m手前の点（折り返し地点）
        point_b_x = cone_b[0] - dir_x * stop_distance
        point_b_y = cone_b[1] - dir_y * stop_distance

        # Pathメッセージを作成
        path_msg = Path()
        path_msg.header = msg.header
        path_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'

        # 往復経路を生成（num_cycles往復 = 2*num_cycles + 1 ポイント）
        waypoints = []
        for i in range(2 * num_cycles + 1):
            if i % 2 == 0:
                # A地点
                waypoints.append((point_a_x, point_a_y, dir_x, dir_y))
            else:
                # B地点
                waypoints.append((point_b_x, point_b_y, -dir_x, -dir_y))

        # 各区間を補間してパスを生成
        for i in range(len(waypoints) - 1):
            x1, y1, _, _ = waypoints[i]
            x2, y2, dir_x_next, dir_y_next = waypoints[i + 1]

            # 区間を補間
            self.add_interpolated_poses(path_msg, x1, y1, x2, y2, path_msg.header, resolution)

        # 最後の点を追加
        last_x, last_y, last_dir_x, last_dir_y = waypoints[-1]
        path_msg.poses.append(self.create_pose(last_x, last_y, path_msg.header, last_dir_x, last_dir_y))

        # パスをパブリッシュ
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f'Published reciprocating path with {len(path_msg.poses)} waypoints ({num_cycles} cycles).')

        self.path_generated = True

    def add_interpolated_poses(self, path_msg, x1, y1, x2, y2, header, resolution=0.3):
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

        # 始点から終点まで補間（終点は含めない、次の区間の始点として追加される）
        for i in range(num_points):
            t = i / float(num_points)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            path_msg.poses.append(self.create_pose(x, y, header, direction_x, direction_y))

    def create_pose(self, x, y, header, direction_x=1.0, direction_y=0.0):
        """
        PoseStampedメッセージを作成
        """
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


def main(args=None):
    rclpy.init(args=args)
    node = TwoConeReciprocate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
