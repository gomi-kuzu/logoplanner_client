from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ─── Launch 引数 ───
        DeclareLaunchArgument('server_host', default_value='localhost',
                              description='LoGoPlanner サーバーの IP アドレス'),
        DeclareLaunchArgument('server_port', default_value='19999',
                              description='LoGoPlanner サーバーのポート番号'),
        DeclareLaunchArgument('server_type', default_value='realworld',
                              description='サーバータイプ: realworld (cmd_list) / simulation (trajectory)'),
        DeclareLaunchArgument('goal_x', default_value='0.0',
                              description='ゴール座標 x [m] (ロボット前方が +x)'),
        DeclareLaunchArgument('goal_y', default_value='0.0',
                              description='ゴール座標 y [m] (ロボット左方が +y)'),
        DeclareLaunchArgument('stop_threshold', default_value='-3.0',
                              description='critic value がこれ以下で停止'),
        DeclareLaunchArgument('robot_type', default_value='holonomic',
                              description='ロボットタイプ: holonomic / differential'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel',
                              description='速度指令のトピック名'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw',
                              description='RGB 画像トピック'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw',
                              description='深度画像トピック'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info',
                              description='CameraInfo トピック'),
        DeclareLaunchArgument('stop_and_go', default_value='false',
                              description='Stop-and-Go モード ON/OFF (realworld モード専用)'),
        DeclareLaunchArgument('stop_and_go_steps', default_value='5',
                              description='1推論あたりの実行ステップ数'),
        DeclareLaunchArgument('stop_wait_time', default_value='0.3',
                              description='推論前の停止待機時間 [s]'),

        # ─── ノード起動 ───
        Node(
            package='logoplanner_client',
            executable='logoplanner_nav_node',
            name='logoplanner_nav_node',
            output='screen',
            parameters=[{
                'server_host': LaunchConfiguration('server_host'),
                'server_port': LaunchConfiguration('server_port'),
                'server_type': LaunchConfiguration('server_type'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'stop_threshold': LaunchConfiguration('stop_threshold'),
                'robot_type': LaunchConfiguration('robot_type'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'stop_and_go': LaunchConfiguration('stop_and_go'),
                'stop_and_go_steps': LaunchConfiguration('stop_and_go_steps'),
                'stop_wait_time': LaunchConfiguration('stop_wait_time'),
            }],
        ),
    ])
