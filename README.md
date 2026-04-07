# logoplanner_client

LoGoPlanner サーバーに RGB-D カメラ画像を送信し、返却されたレスポンスから速度指令 (`geometry_msgs/Twist`) を `cmd_vel` トピックにパブリッシュする ROS2 パッケージです。

**目標設定方法:**
- **Action（推奨）**: `/navigate_to_goal` アクションで動的に目標を設定・キャンセル・進捗確認
- パラメータ: 起動時の初期目標（Action未使用時のみ）

## アーキテクチャ

```
┌─────────────────────────────────────────────────────────┐
│  logoplanner_nav_node                                    │
│                                                          │
│  [Action Server]                                         │
│    /navigate_to_goal (NavigateToGoal)                   │
│    → ゴール設定・進捗フィードバック・キャンセル         │
│                        ↕ lock                            │
│  [計画スレッド]                                          │
│    RGB+Depth → HTTP POST /pointgoal_step                │
│    → trajectory / cmd_list を更新                        │
│                        ↕ lock                            │
│  [制御タイマー (10Hz)]                                   │
│    realworld:   cmd_list → 速度指令                     │
│    simulation:  trajectory → P制御                       │
└──────────────────────────────────────────────────────────┘
         ↑ Subscribe                  ↓ Publish
   /camera/color/image_raw        cmd_vel
   /camera/depth/image_raw        (geometry_msgs/Twist)
   /camera/color/camera_info
```

## 依存パッケージ

- `ros2_astra_camera` (RGB-D カメラドライバ)
- Python: `requests`, `numpy`, `opencv-python`

## ビルド

```bash
cd ~/humble_ws
colcon build --packages-select logoplanner_client
source install/setup.bash
```

## 使い方

### 1. カメラノード起動

```bash
ros2 launch astra_camera astra_pro.launch.xml
```

### 2. LoGoPlanner ナビゲーションノード起動

```bash
ros2 launch logoplanner_client logoplanner_nav.launch.py
```

### 3. ゴールを送信（Action）

**別ターミナルで:**

```bash
# CLI でゴール送信
ros2 action send_goal /navigate_to_goal logoplanner_client/action/NavigateToGoal "{goal_x: 2.0, goal_y: 0.0}" --feedback

# Python スクリプト例
python3 << EOF
import rclpy
from rclpy.action import ActionClient
from logoplanner_client.action import NavigateToGoal

rclpy.init()
node = rclpy.create_node('goal_sender')
client = ActionClient(node, NavigateToGoal, '/navigate_to_goal')
client.wait_for_server()

goal = NavigateToGoal.Goal()
goal.goal_x = 2.0
goal.goal_y = 0.5

future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future)
print('Goal sent!')
node.destroy_node()
rclpy.shutdown()
EOF
```

### 4. キャンセル

```bash
# 実行中のゴールをキャンセル
ros2 action send_goal /navigate_to_goal logoplanner_client/action/NavigateToGoal "{goal_x: 0.0, goal_y: 0.0}" --feedback
# または Ctrl+C
```

## パラメータ一覧

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `server_host` | `192.168.11.13` | LoGoPlanner サーバーの IP |
| `server_port` | `19999` | サーバーのポート番号 |
| `server_type` | `realworld` | `realworld` (cmd_list) / `simulation` (trajectory) |
| `goal_x` | `1.0` | ゴール x [m] (前方が +x) |
| `goal_y` | `0.0` | ゴール y [m] (左が +y) |
| `stop_threshold` | `-3.0` | critic value がこれ以下で停止 |
| `kp_v` | `1.0` | 線速度ゲイン |
| `kp_w` | `1.0` | 横ずれ→角速度ゲイン |
| `kp_theta` | `1.0` | 角度差→角速度ゲイン |
| `max_linear_x` | `0.5` | 最大線速度 x [m/s] |
| `max_linear_y` | `0.5` | 最大線速度 y [m/s] (ホロノミック用) |
| `max_angular_z` | `1.0` | 最大角速度 [rad/s] |
| `robot_type` | `holonomic` | `holonomic` or `differential` |
| `control_rate` | `10.0` | cmd_vel パブリッシュ周期 [Hz] |
| `planning_interval` | `0.1` | サーバー問い合わせ間隔 [s] |
| `rgb_topic` | `/camera/color/image_raw` | RGB トピック |
| `depth_topic` | `/camera/depth/image_raw` | 深度トピック |
| `camera_info_topic` | `/camera/color/camera_info` | CameraInfo トピック |
| `cmd_vel_topic` | `cmd_vel` | 速度指令トピック |

## ロボットタイプ

### `holonomic` (メカナムホイール等)

```
vx = kp_v * dx
vy = kp_w * dy
wz = kp_theta * dtheta
```

### `differential` (差動二輪)

```
v     = kp_v * dx
omega = kp_w * dy + kp_theta * dtheta
```

## 前提条件

- LoGoPlanner サーバーが `192.168.11.13:19999` で起動済みであること
- astra_camera ノードが RGB-D 画像を配信していること

## 注意事項

- `trajectory` はロボット座標系での **累積変位** (速度ではない)
- 深度画像は astra_camera の 16UC1 (mm) → LoGoPlanner の m×10000 に自動変換
- サーバーの推論に時間がかかる場合でも、制御タイマーは前回の trajectory を使い回して一定周期で cmd_vel を送出
- ゴール到達後は自動的に停止コマンドを送出
