from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # AI ボードノード
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen',
        ),

        # AO ボードノード
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='ao1608llpe_node',
            output='screen',
        ),

        # エンコーダノード
        Node(
            package='control_box',
            executable='cnt3204mtlpe_test',
            name='cnt3204mtlpe_node',
            output='screen',
        ),

        # 信号変換・フィルタノード
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter_cyl',
            name='sensor_interpreter_node',
            output='screen',
            parameters=[{
                'head_pressure_index': 0,
                'rod_pressure_index':  1,
                'cutoff_hz_pressure' : 10.0,
            }],
        ),

        # シリンダ位置制御ノード
        Node(
            package='cylinder_exp',
            executable='pos_controller',
            name='cylinder_position_controller_node',
            output='screen',
            parameters=[{

                # ハードウェア
                'ch_head': 0,
                'ch_rod':  1,

                # ループ周期
                'outer_rate_hz': 500.0,
                'inner_rate_hz': 1000.0,

                # 正弦波軌道
                # x_0（PAM自然位置）が最大端。そこから -2A まで縮む。
                # 全ストローク = 2 * sine_amplitude_m 
                'sine_amplitude_m': 0.020,
                'sine_freq_hz':     1.0,

                # 振幅ランプ（0 → sine_amplitude_m に到達する速度）
                # 0.005 m/s → 7 mm 振幅に約 1.4 秒で到達
                'sine_amp_ramp_rate_m_s': 0.0005,

                # 圧力
                'base_pressure_kpa':   150.0,
                'supply_pressure_kpa': 640.0,

                # 位置ループ PID（外側）
                'pos_kp': 1500.0,
                'pos_ki': 30.0,
                'pos_kd': 0.0,
                'pos_td': 1.0,
                'pos_output_limit': 1000.0,

                # 圧力ループ PI（内側）
                'pres_kp': 0.016, #0.015
                'pres_ki': 0.002,
                'pres_kd': 0.0,
                'pres_td': 0.01,
                'pres_output_limit': 4.9,

                # ホーミング（両室排気 → PAM自然位置を検出）
                'homing_settle_threshold': 0.002,
                'homing_settle_duration':  2.0,
                'homing_startup_wait':     2.5,

                # ロードセル補償（まずはオフで試す）
                'use_loadcell_compensation': False,
                'loadcell_ff_gain': 1.0,
                'loadcell_timeout_s': 3.2,
            }],
        ),
    ])
