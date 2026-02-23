# coverage_planner - カバレッジパス生成パッケージ

コーン群が囲むエリア内を走行するための経路を生成するROS 2パッケージです。

## 概要

**役割:** カバレッジパスプランニング（経路計画）

このパッケージは以下の処理を行います：
1. `/cone_area` (PolygonStamped) をサブスクライブ
2. 多角形を安全マージン分縮小
3. ロボットの現在位置と向きを取得（TF2）
4. **前方にある辺の真ん中**を開始位置として設定
5. 矩形の外周を巡回する経路を生成
6. `/coverage_path` (Path) として発行

## 主要な機能

### 前方の辺からスタート
- ロボットの現在位置と向き（yaw）を取得
- 各辺の中点とロボットの前方方向の内積を計算
- **最も前方にある辺の真ん中**を開始位置に設定
- これにより、ロボットが自然に経路に乗りやすくなります

### 安全マージン
- **safety_margin: 0.75m** - コーン群から0.75m内側に経路を生成
- 計算根拠:
  - インフレーション半径: 0.3m
  - ロボット対角線半径: 0.354m
  - オーバーシュート余裕: 0.1m
  - 合計: 約0.754m
- コーンエリアから出ないように安全性を確保

### 高解像度経路
- **ウェイポイント間隔: 0.1m** - 密なウェイポイントで滑らかな経路追従
- 矩形の頂点でも曲がりやすい
- DWBコントローラーとの相性が良い

### 矩形経路パターン
- 縮小されたポリゴンの外周を巡回
- 閉じた経路（開始位置に戻る）
- アプローチポイントから滑らかに経路に進入

## 依存関係

### ROS 2パッケージ
```bash
sudo apt install ros-humble-geometry-msgs ros-humble-nav-msgs \
                 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

### Python
```bash
# 標準ライブラリのみ使用（math, rclpy）
```

## ビルド

```bash
cd ~/mirs_ws
colcon build --packages-select coverage_planner --symlink-install
source install/setup.bash
```

## 起動方法

### 単体起動
```bash
ros2 run coverage_planner rectangle_generator
```

### フルシステム起動（推奨）
```bash
# コーン検出、経路生成、ナビゲーションをすべて起動
ros2 launch mirs system_bringup.launch.py
```

## トピック

### サブスクライブ
- `/cone_area` (geometry_msgs/PolygonStamped) - コーン群の凸包エリア

### パブリッシュ
- `/coverage_path` (nav_msgs/Path) - 生成された経路（0.1m間隔のウェイポイント）

## パラメータ

### safety_margin
- **デフォルト**: 0.75m
- **説明**: コーン群の凸包から内側に縮小する距離
- **調整方法**:
  ```python
  # rectangle_generator.py 15行目
  self.declare_parameter('safety_margin', 0.75)
  ```

### resolution（経路補間間隔）
- **デフォルト**: 0.1m
- **説明**: ウェイポイント間の距離
- **調整方法**:
  ```python
  # rectangle_generator.py 173行目
  def add_interpolated_poses(self, path_msg, x1, y1, x2, y2, header, resolution=0.1):
  ```

## アルゴリズム

### 1. ポリゴン縮小（Offset Polygon）
```
元のポリゴン → 各頂点を内側に safety_margin だけオフセット → 縮小ポリゴン
```

### 2. 前方の辺を検出
```
for each 辺:
    辺の中点を計算
    ロボット → 中点のベクトルと、ロボットの前方ベクトルの内積を計算

内積が最大の辺を選択 → これが「前方の辺」
```

### 3. 経路生成
```
1. アプローチポイント（開始位置の手前1m）
2. 開始位置（前方の辺の真ん中）
3. 辺の終点
4. 次の辺へ...（矩形を一周）
5. 開始位置に戻る（閉じた経路）
```

### 4. 補間
各辺を0.1m間隔で補間してウェイポイントを生成

## 使用例

### 典型的なワークフロー
1. コーン検出: `cone_detector` → `/cone_area`
2. 経路生成: `rectangle_generator` → `/coverage_path`
3. ナビゲーション: Nav2 FollowPath → `/cmd_vel`

### 生成される経路の例
```
コーン配置（約1.5m × 1.5m四角形）
↓
safety_margin 0.75m で縮小
↓
経路サイズ: 約0.45m × 0.45m
ウェイポイント数: 約20-40個（0.1m間隔）
```

## RViz可視化

RViz2で以下のトピックを追加：
- `/cone_area` (Polygon) - コーン群のエリア（青線）
- `/coverage_path` (Path) - 生成された経路（緑線）

表示設定：
- Path → Line Style: Lines
- Path → Color: Green
- Polygon → Color: Blue

## トラブルシューティング

### 経路が生成されない
**原因**: コーン検出されていない、または`/cone_area`が発行されていない
```bash
# /cone_area が発行されているか確認
ros2 topic echo /cone_area

# コーン検出ノードが起動しているか確認
ros2 node list | grep cone
```

### 経路が小さすぎる
**原因**: safety_margin が大きすぎる
**解決策**: `rectangle_generator.py`の`safety_margin`を小さくする（例: 0.5m）
**注意**: 小さくしすぎるとロボットがコーンエリアから出る可能性があります

### Inner polygon is invalid or too small
**原因**: コーンエリアが小さすぎて、safety_margin 分縮小すると消滅する
**解決策**:
1. コーンをより広い範囲に配置
2. safety_marginを小さくする

### ロボットが開始位置に到達できない
**原因**: 開始位置がロボットから遠すぎる、または障害物がある
**解決策**:
- behavior_tree で NavigateToPose をスキップして直接 FollowPath を開始
- 現在の実装ではスタート地点への移動はスキップされています

## 関連パッケージ

- **cone_detector**: コーンを検出して`/cone_area`を生成
- **bt_pkg**: ビヘイビアツリーでミッション実行（経路追従を5周繰り返し）
- **mirs**: Nav2コントローラーで経路追従

## ファイル構成

```
coverage_planner/
├── coverage_planner/
│   └── rectangle_generator.py   # メインノード
├── package.xml
├── setup.py
└── README.md
```

## 開発者向け情報

### 経路生成ロジックのカスタマイズ

#### 異なるパターンを実装する場合
1. `rectangle_generator.py`をコピー
2. `area_callback`メソッドの経路生成ロジックを変更
3. `setup.py`にエントリーポイントを追加

#### 例: ジグザグパターン
```python
# zigzag_generator.py として実装
# エリア内部をジグザグに走行する経路を生成
```

### TF2の使用
```python
# ロボットの現在位置と向きを取得
transform = self.tf_buffer.lookup_transform(
    'odom',       # target frame
    'base_link',  # source frame
    rclpy.time.Time())
```

## ライセンス

このパッケージのライセンスについては、LICENSEファイルを参照してください。
