#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LoGoPlanner ナビゲーションノード

RGB-D カメラ画像を LoGoPlanner サーバーに送信し、
返却された trajectory から速度指令 (geometry_msgs/Twist) を
cmd_vel トピックにパブリッシュする ROS2 ノード。

アーキテクチャ:
  - メインスレッド: ROS2 spin (サブスクライバ・パブリッシャ・タイマー)
  - 計画スレッド : サーバーへの HTTP 問い合わせ (ブロッキング)
  計画スレッドが最新の trajectory を更新し、
  制御タイマーが前回の trajectory を使い回すことで
  推論遅延に左右されず一定周期で速度指令を送出する。
"""

import io
import json
import threading
import time
from typing import Optional

import cv2
import numpy as np
import requests
import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from logoplanner_client.action import NavigateToGoal


class LogoPlannerNavNode(Node):
    """LoGoPlanner クライアント兼 cmd_vel パブリッシャノード."""

    def __init__(self) -> None:
        super().__init__('logoplanner_nav_node')

        # ─── ROS パラメータ宣言 ───
        self.declare_parameter('server_host', 'localhost')
        self.declare_parameter('server_port', 19999)

        # サーバータイプ: "realworld" or "simulation"
        # realworld  → logoplanner_realworld_server (cmd_list を返す、MPC内蔵)
        # simulation → logoplanner_server (trajectory を返す、P制御必要)
        self.declare_parameter('server_type', 'realworld')

        # ゴール座標 (ロボット座標系: 前方=+x, 左=+y [m])
        # パラメータは初期値のみ。Actionで上書き可能
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)

        # 停止判定閾値 (critic value がこれ以下で停止)
        self.declare_parameter('stop_threshold', -3.0)

        # P 制御ゲイン
        self.declare_parameter('kp_v', 1.0)       # 線速度ゲイン (dx → v)
        self.declare_parameter('kp_w', 1.0)       # 横ずれ → 角速度ゲイン
        self.declare_parameter('kp_theta', 1.0)   # 角度差 → 角速度ゲイン

        # 速度リミット
        self.declare_parameter('max_linear_x', 0.5)   # [m/s]
        self.declare_parameter('max_linear_y', 0.5)    # [m/s] (ホロノミック用)
        self.declare_parameter('max_angular_z', 1.0)   # [rad/s]

        # ロボットタイプ: "holonomic" or "differential"
        self.declare_parameter('robot_type', 'holonomic')

        # 制御周期 [Hz]
        self.declare_parameter('control_rate', 10.0)

        # 計画ループの sleep [s]
        self.declare_parameter('planning_interval', 0.1)

        # Stop-and-Go モード (realworld モード専用)
        # True にすると: 推論前にロボットを停止 → 推論 → N ステップ実行 → 繰り返し
        self.declare_parameter('stop_and_go', False)
        # 1推論あたりに実行する cmd_list のステップ数
        self.declare_parameter('stop_and_go_steps', 5)
        # 停止コマンド送出後、ロボットが実際に止まるまでの待機時間 [s]
        self.declare_parameter('stop_wait_time', 0.3)

        # カメラトピック名
        # Astra Pro のデフォルト設定
        # RealSense D435 の場合:
        #   rgb_topic: /camera/camera/color/image_raw
        #   depth_topic: /camera/camera/depth/image_rect_raw
        #   camera_info_topic: /camera/camera/color/camera_info
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        # cmd_vel トピック名
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        # ─── パラメータ取得 ───
        self._server_host: str = self.get_parameter('server_host').value
        self._server_port: int = self.get_parameter('server_port').value
        self._server_type: str = self.get_parameter('server_type').value
        if self._server_type not in ('realworld', 'simulation'):
            self.get_logger().error(
                f"Invalid server_type '{self._server_type}'. "
                "Must be 'realworld' or 'simulation'. Defaulting to 'realworld'.")
            self._server_type = 'realworld'
        self._goal_x: float = self.get_parameter('goal_x').value
        self._goal_y: float = self.get_parameter('goal_y').value
        self._stop_threshold: float = self.get_parameter('stop_threshold').value
        self._kp_v: float = self.get_parameter('kp_v').value
        self._kp_w: float = self.get_parameter('kp_w').value
        self._kp_theta: float = self.get_parameter('kp_theta').value
        self._max_vx: float = self.get_parameter('max_linear_x').value
        self._max_vy: float = self.get_parameter('max_linear_y').value
        self._max_wz: float = self.get_parameter('max_angular_z').value
        self._robot_type: str = self.get_parameter('robot_type').value
        self._control_rate: float = self.get_parameter('control_rate').value
        self._planning_interval: float = self.get_parameter('planning_interval').value
        self._stop_and_go: bool = self.get_parameter('stop_and_go').value
        self._stop_and_go_steps: int = self.get_parameter('stop_and_go_steps').value
        self._stop_wait_time: float = self.get_parameter('stop_wait_time').value

        # ─── 内部状態 ───
        self._lock = threading.Lock()

        # 最新の画像データ (保護対象)
        self._latest_rgb: Optional[np.ndarray] = None      # RGB uint8
        self._latest_depth: Optional[np.ndarray] = None    # uint16 [mm]
        self._camera_intrinsic: Optional[np.ndarray] = None

        # 最新の計画結果 (計画スレッドが更新)
        # simulation モード用
        self._trajectory: Optional[np.ndarray] = None
        self._all_values: Optional[np.ndarray] = None
        # realworld モード用
        self._cmd_list: Optional[np.ndarray] = None   # (N, 3) [vx, vy, wz_deg]
        self._cmd_index: int = 0                       # cmd_list 内の現在位置

        self._reached_goal: bool = False
        self._server_initialized: bool = False
        # Stop-and-Go: True の間は制御タイマーがゼロ速度を publish する
        self._stop_for_inference: bool = False
        self._start_time: Optional[float] = None  # ナビゲーション開始時刻
        self._action_active: bool = False  # アクション実行中フラグ

        # ─── パブリッシャ ───
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ─── サブスクライバ ───
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        self.create_subscription(Image, rgb_topic, self._rgb_callback, 10)
        self.create_subscription(Image, depth_topic, self._depth_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, 10)

        # ─── 制御タイマー ───
        control_period = 1.0 / self._control_rate
        self._control_timer = self.create_timer(control_period, self._control_callback)

        # ─── Actionサーバー ───
        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            execute_callback=self._execute_navigate,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )

        # ─── 計画スレッド ───
        self._planning_thread = threading.Thread(target=self._planning_loop, daemon=True)
        self._planning_thread.start()

        sag_info = (
            f'stop_and_go=ON (steps={self._stop_and_go_steps}, wait={self._stop_wait_time}s)'
            if self._stop_and_go else 'stop_and_go=OFF'
        )
        self.get_logger().info(
            f'LoGoPlanner Nav Node started  '
            f'server={self._server_host}:{self._server_port}  '
            f'server_type={self._server_type}  '
            f'robot_type={self._robot_type}  '
            f'{sag_info}  '
            f'Action server: /navigate_to_goal'
        )

    # ================================================================
    #  Actionサーバーコールバック
    # ================================================================
    def _goal_callback(self, goal_request):
        """新しいゴールリクエストの受付判定."""
        self.get_logger().info(
            f'Received goal request: x={goal_request.goal_x:.2f}, y={goal_request.goal_y:.2f}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """ゴールキャンセルリクエストの受付判定."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def _execute_navigate(self, goal_handle):
        """ナビゲーションゴールの実行."""
        self.get_logger().info('Executing navigate_to_goal...')
        goal = goal_handle.request
        feedback_msg = NavigateToGoal.Feedback()

        # ゴール座標を更新
        with self._lock:
            self._goal_x = goal.goal_x
            self._goal_y = goal.goal_y
            self._reached_goal = False
            self._start_time = time.time()
            self._action_active = True

        # 環境メモリをクリア
        self._server_reset_env(env_id=0)

        # ゴール到達を待つ (0.1秒ごとにチェック、2秒に1回フィードバック)
        loop_count = 0
        while rclpy.ok():
            # 早期キャンセルチェック (0.1秒ごと)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateToGoal.Result()
                result.success = False
                result.message = 'Goal canceled by user'
                result.final_distance = self._estimate_distance_to_goal()
                with self._lock:
                    self._reached_goal = True
                    self._action_active = False
                self.get_logger().info('Goal canceled')
                return result

            # ゴール到達チェック
            with self._lock:
                reached = self._reached_goal
                start_time = self._start_time

            if reached:
                # ゴール到達
                result = NavigateToGoal.Result()
                result.success = True
                result.message = 'Goal reached successfully'
                result.final_distance = 0.0
                goal_handle.succeed()
                with self._lock:
                    self._action_active = False
                self.get_logger().info('Goal succeeded')
                return result

            # フィードバック更新 (2秒に1回 = 0.1秒×20回)
            if loop_count % 20 == 0:
                feedback_msg.distance_to_goal = self._estimate_distance_to_goal()
                feedback_msg.elapsed_time = time.time() - start_time if start_time else 0.0
                feedback_msg.current_x = 0.0  # TODO: オドメトリがあれば更新
                feedback_msg.current_y = 0.0
                goal_handle.publish_feedback(feedback_msg)

            loop_count += 1
            time.sleep(0.1)  # 0.1秒ごとにキャンセルチェック

        # rclpy.ok() が False になった場合
        result = NavigateToGoal.Result()
        result.success = False
        result.message = 'Navigation aborted (node shutdown)'
        result.final_distance = self._estimate_distance_to_goal()
        goal_handle.abort()
        with self._lock:
            self._action_active = False
        return result

    def _estimate_distance_to_goal(self) -> float:
        """現在位置からゴールまでの推定距離 [m]."""
        with self._lock:
            goal_x = self._goal_x
            goal_y = self._goal_y
        # TODO: オドメトリがあれば実際の位置を使う
        # 現在は単純にゴール座標のノルムを返す
        return float(np.sqrt(goal_x**2 + goal_y**2))

    # ================================================================
    #  サブスクライバコールバック
    # ================================================================
    def _rgb_callback(self, msg: Image) -> None:
        """RGB 画像を受信して内部バッファに保存 (RGB形式)."""
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            # サーバーはRGB順序を要求するため、RGB形式で保存
            if msg.encoding == 'rgb8':
                # rgb8 はそのまま
                pass
            elif msg.encoding == 'bgr8':
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            elif msg.encoding == 'bgra8':
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            elif msg.encoding == 'rgba8':
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
            with self._lock:
                self._latest_rgb = img
        except Exception as e:
            self.get_logger().error(f'RGB conversion failed: {e}')

    def _depth_callback(self, msg: Image) -> None:
        """深度画像を受信して内部バッファに保存.

        astra_camera は 16UC1 (単位: mm) で配信する。
        LoGoPlanner サーバーは 深度[m] × 10000 の uint16 PNG を期待するため、
        送信時に変換する。
        """
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            elif msg.encoding == '32FC1':
                depth_f = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
                depth = (depth_f * 1000.0).astype(np.uint16)  # m → mm
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
                return
            with self._lock:
                self._latest_depth = depth.copy()
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """CameraInfo から内部パラメータ行列を取得 (初回のみ)."""
        if self._camera_intrinsic is not None:
            return
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        k = np.array(msg.k).reshape(3, 3)
        with self._lock:
            self._camera_intrinsic = k
        self.get_logger().info(f'Camera intrinsic received:\n{k}')

    # ================================================================
    #  サーバー通信ユーティリティ
    # ================================================================
    @property
    def _base_url(self) -> str:
        return f'http://{self._server_host}:{self._server_port}'

    def _server_reset(self, intrinsic: np.ndarray) -> bool:
        """POST /navigator_reset でエージェントを初期化."""
        url = f'{self._base_url}/navigator_reset'
        payload = {
            'intrinsic': intrinsic.tolist(),
            'stop_threshold': self._stop_threshold,
            'batch_size': 1,
        }
        try:
            resp = requests.post(url, json=payload, timeout=10.0)
            resp.raise_for_status()
            result = resp.json()
            self.get_logger().info(f'navigator_reset OK: {result}')
            return True
        except Exception as e:
            self.get_logger().error(f'navigator_reset failed: {e}')
            return False

    def _server_reset_env(self, env_id: int = 0) -> bool:
        """POST /navigator_reset_env で環境メモリをクリア."""
        url = f'{self._base_url}/navigator_reset_env'
        try:
            resp = requests.post(url, json={'env_id': env_id}, timeout=10.0)
            resp.raise_for_status()
            return True
        except Exception as e:
            self.get_logger().error(f'navigator_reset_env failed: {e}')
            return False

    def _server_pointgoal_step(
        self, rgb: np.ndarray, depth: np.ndarray
    ) -> Optional[dict]:
        """POST /pointgoal_step で 1 ステップ推論.

        Parameters
        ----------
        rgb : np.ndarray
            RGB uint8 画像 (H, W, 3)
        depth : np.ndarray
            深度 uint16 画像 (H, W) [mm]

        Returns
        -------
        dict or None
            サーバーレスポンス (trajectory, all_values, etc.)
        """
        url = f'{self._base_url}/pointgoal_step'

        # --- RGB → JPEG bytes ---
        # cv2.imencode はデフォルトでBGR想定のため、RGB→BGRに変換してエンコード
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        _, rgb_encoded = cv2.imencode('.jpg', rgb_bgr)
        rgb_bytes = io.BytesIO(rgb_encoded.tobytes())

        # --- Depth → PNG bytes ---
        # astra は mm 単位の uint16。
        # simulation サーバー: m×10000 を期待 → mm×10 (サーバー側で /10000.0)
        # realworld サーバー : mm をそのまま期待 (サーバー側で /1000.0)
        if self._server_type == 'simulation':
            depth_scaled = np.clip(depth.astype(np.float64) * 10.0, 0, 65535).astype(np.uint16)
        else:
            depth_scaled = depth  # mm のまま送信
        _, depth_encoded = cv2.imencode('.png', depth_scaled)
        depth_bytes = io.BytesIO(depth_encoded.tobytes())

        files = {
            'image': ('image.jpg', rgb_bytes.getvalue(), 'image/jpeg'),
            'depth': ('depth.png', depth_bytes.getvalue(), 'image/png'),
        }
        data = {
            'goal_data': json.dumps({
                'goal_x': [self._goal_x],
                'goal_y': [self._goal_y],
            }),
        }

        try:
            resp = requests.post(url, files=files, data=data, timeout=30.0)
            resp.raise_for_status()
            return resp.json()
        except Exception as e:
            self.get_logger().error(f'pointgoal_step failed: {e}')
            return None

    # ================================================================
    #  計画スレッド
    # ================================================================
    def _planning_loop(self) -> None:
        """バックグラウンドで繰り返しサーバーに問い合わせる."""
        self.get_logger().info('Planning thread started, waiting for camera data...')

        # カメラ intrinsic が届くまで待機
        while rclpy.ok():
            with self._lock:
                intrinsic = self._camera_intrinsic
            if intrinsic is not None:
                break
            time.sleep(0.5)

        # サーバー初期化
        while rclpy.ok():
            if self._server_reset(intrinsic):
                self._server_initialized = True
                break
            self.get_logger().warn('Retrying navigator_reset in 3s...')
            time.sleep(3.0)

        # 環境メモリクリア
        self._server_reset_env(env_id=0)

        self.get_logger().info('Planning loop running.')

        while rclpy.ok():
            # アクション実行中かつゴール未達成の場合のみ計画実行
            with self._lock:
                action_active = self._action_active
                reached = self._reached_goal
            
            if not action_active:
                self.get_logger().debug('Planning paused: waiting for action goal')
                time.sleep(0.5)
                continue
            
            if reached:
                self.get_logger().debug('Planning paused: goal reached')
                time.sleep(0.5)
                continue

            # 最新画像の取得
            with self._lock:
                rgb = self._latest_rgb
                depth = self._latest_depth

            if rgb is None or depth is None:
                time.sleep(0.1)
                continue

            # Stop-and-Go: 問い合わせ前にロボットを停止
            if self._stop_and_go and self._server_type == 'realworld':
                with self._lock:
                    self._stop_for_inference = True
                time.sleep(self._stop_wait_time)

            # サーバー問い合わせ
            with self._lock:
                current_goal_x = self._goal_x
                current_goal_y = self._goal_y
            self.get_logger().debug(
                f'Querying server with goal=({current_goal_x:.2f}, {current_goal_y:.2f})'
            )
            result = self._server_pointgoal_step(rgb, depth)
            if result is None:
                time.sleep(self._planning_interval)
                continue

            if self._server_type == 'realworld':
                # realworld サーバー: cmd_list (MPC計算済み速度指令)
                if 'cmd_list' in result:
                    cmd_list = np.array(result['cmd_list'])  # (N, 3) [vx, vy, wz_deg]
                    with self._lock:
                        self._cmd_list = cmd_list
                        self._cmd_index = 0
                        self._stop_for_inference = False  # 実行再開
                    self.get_logger().debug(
                        f'Received cmd_list: {len(cmd_list)} commands')

                    if self._stop_and_go:
                        # Stop-and-Go: N ステップ消費完了まで待ってから次の推論へ
                        target_steps = min(self._stop_and_go_steps, len(cmd_list))
                        self.get_logger().debug(
                            f'[Stop-and-Go] Executing {target_steps} steps...')
                        while rclpy.ok():
                            with self._lock:
                                idx = self._cmd_index
                                reached = self._reached_goal
                            if reached or idx >= target_steps:
                                break
                            time.sleep(0.02)
                        # ステップ実行完了 → 次の推論のために停止
                        with self._lock:
                            self._stop_for_inference = True
                        self.get_logger().debug(
                            f'[Stop-and-Go] Stopping for next inference...')
                        time.sleep(self._stop_wait_time)
                        continue  # planning_interval を挟まずすぐ次の推論へ
                else:
                    self.get_logger().warn(
                        f'realworld mode but response has no cmd_list. '
                        f'Keys: {list(result.keys())}')
            else:
                # simulation サーバー: trajectory + all_values
                trajectory = np.array(result['trajectory'])
                all_values = np.array(result['all_values'])

                with self._lock:
                    self._trajectory = trajectory
                    self._all_values = all_values

                # 停止判定
                if np.max(all_values) < self._stop_threshold:
                    self.get_logger().info(
                        f'Goal reached! max_value={np.max(all_values):.3f} < '
                        f'threshold={self._stop_threshold}'
                    )
                    with self._lock:
                        self._reached_goal = True

            time.sleep(self._planning_interval)

    # ================================================================
    #  制御タイマーコールバック
    # ================================================================
    def _control_callback(self) -> None:
        """タイマー周期で速度指令を計算・パブリッシュ."""
        if self._server_type == 'realworld':
            self._control_callback_realworld()
        else:
            self._control_callback_simulation()

    def _control_callback_realworld(self) -> None:
        """realworld モード: cmd_list から速度指令を順次送出.

        cmd_list は [vx, vy, wz(deg/s)] のリスト。
        制御タイマーの各 tick で 1 コマンドずつ消費する。
        全コマンド消費後は最後のコマンドを保持（次の cmd_list 到着まで）。
        Stop-and-Go モード時は _stop_for_inference=True の間ゼロ速度を送出。
        """
        twist = Twist()

        with self._lock:
            cmd_list = self._cmd_list
            cmd_idx = self._cmd_index
            reached = self._reached_goal
            stop_for_inference = self._stop_for_inference

        # 推論待ち停止 or ゴール到達 → ゼロ速度
        if stop_for_inference or reached:
            self._cmd_pub.publish(twist)
            if stop_for_inference:
                self.get_logger().debug('[Stop-and-Go] Holding stop for inference...')
            return

        if cmd_list is None:
            self._cmd_pub.publish(twist)
            return

        # 現在のコマンドを取得
        idx = min(cmd_idx, len(cmd_list) - 1)
        vx, vy, wz_deg = float(cmd_list[idx][0]), float(cmd_list[idx][1]), float(cmd_list[idx][2])

        # wz は deg/s → rad/s に変換
        wz_rad = np.deg2rad(wz_deg)

        # クランプ
        vx = max(-self._max_vx, min(self._max_vx, vx))
        vy = max(-self._max_vy, min(self._max_vy, vy))
        wz_rad = max(-self._max_wz, min(self._max_wz, wz_rad))

        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz_rad

        self._cmd_pub.publish(twist)

        # インデックスを進める
        with self._lock:
            if self._cmd_index < len(cmd_list) - 1:
                self._cmd_index += 1

        self.get_logger().debug(
            f'cmd_vel [realworld {idx}/{len(cmd_list)}]: '
            f'vx={vx:.3f} vy={vy:.3f} wz={wz_rad:.3f}rad/s ({wz_deg:.1f}deg/s)'
        )

    def _control_callback_simulation(self) -> None:
        """simulation モード: trajectory から P制御で速度指令を計算."""
        twist = Twist()

        with self._lock:
            trajectory = self._trajectory
            reached = self._reached_goal

        if trajectory is None or reached:
            self._cmd_pub.publish(twist)
            return

        # trajectory shape: (batch, predict_size=24, 3)
        # waypoint[0]: 1 ステップ先の累積変位 [dx, dy, dtheta]
        waypoint = trajectory[0][0]
        dx, dy, dtheta = float(waypoint[0]), float(waypoint[1]), float(waypoint[2])

        if self._robot_type == 'holonomic':
            vx = self._kp_v * dx
            vy = self._kp_w * dy
            wz = self._kp_theta * dtheta

            vx = max(-self._max_vx, min(self._max_vx, vx))
            vy = max(-self._max_vy, min(self._max_vy, vy))
            wz = max(-self._max_wz, min(self._max_wz, wz))

            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
        else:
            v = self._kp_v * dx
            omega = self._kp_w * dy + self._kp_theta * dtheta

            v = max(-self._max_vx, min(self._max_vx, v))
            omega = max(-self._max_wz, min(self._max_wz, omega))

            twist.linear.x = v
            twist.angular.z = omega

        self._cmd_pub.publish(twist)

        self.get_logger().debug(
            f'cmd_vel [simulation]: vx={twist.linear.x:.3f} vy={twist.linear.y:.3f} '
            f'wz={twist.angular.z:.3f}  wp=[{dx:.4f},{dy:.4f},{dtheta:.4f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LogoPlannerNavNode()
    
    # MultiThreadedExecutorを使用してアクション実行中もタイマーコールバックを実行
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 停止コマンドを送出してからシャットダウン
        stop_twist = Twist()
        node._cmd_pub.publish(stop_twist)
        node.get_logger().info('Shutting down, sending stop command.')
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
