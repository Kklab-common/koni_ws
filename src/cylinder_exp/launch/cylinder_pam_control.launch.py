from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime
import os


def generate_launch_description():
    bag_dir = os.path.expanduser(
        f'~/koni_log/{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-s', 'mcap',
            '/actuators/cylinder_valves',
            '/sensors/cylinder_position',
            '/sensors/head_pressure',
            '/sensors/rod_pressure',
            '/sensors/loadcell_force',
            '/sensors/pam_pressure',
            '/actuators/pam_valve',
            '/debug/pam_valve_output_V',
        ],
        output='screen',
    )

    return LaunchDescription([
        bag_record,
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

        # カウンタボードノード
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
                'head_pressure_index':  1,
                'rod_pressure_index':   3,
                'loadcell_plus_index':  2,
                'loadcell_minus_index': 3,
                'pam_pressure_index':   4,
                'cutoff_hz_pressure':   10.0,
            }],
        ),

        # シリンダ位置制御ノード 
        Node(
            package='cylinder_exp',
            executable='pos_controller',
            name='cylinder_position_controller_node',
            output='screen',
            parameters=[{
                # AOボードとの接続チャンネル
                'ch_head': 1,
                'ch_rod':  3,

                # ループ周期
                'outer_rate_hz': 500.0,
                'inner_rate_hz': 1000.0,

                # 正弦波軌道
                'sine_amplitude_m':       0.020,
                'sine_freq_hz':           1.0,
                'sine_amp_ramp_rate_m_s': 0.0005,

                # 圧力
                'base_pressure_kpa':   150.0,
                'supply_pressure_kpa': 600.0,

                # 位置ループ PID
                'pos_kp': 100.0,
                'pos_ki': 0.0,
                'pos_kd': 0.0,
                'pos_td': 1.0,
                'pos_output_limit': 1000.0,

                # 圧力ループ PI
                'pres_kp': 0.0006,
                'pres_ki': 0.00,
                'pres_kd': 0.0,
                'pres_td': 0.01,
                'pres_output_limit': 4.9,

                # ホーミング
                'homing_settle_threshold': 0.002,
                'homing_settle_duration':  2.0,
                'homing_startup_wait':     2.5,

                # ロードセル補償
                'use_loadcell_compensation': False,
                'loadcell_ff_gain':          1.0,
                'loadcell_timeout_s':        3.2,
            }],
        ),

        # PAM 定圧制御ノード
        Node(
            package='cylinder_exp',
            executable='pam_const_pressure_controller',
            name='pam_const_pressure_controller_node',
            output='screen',
            parameters=[{
                'target_pressure_kpa': 100.0,
                'kp':                  0.0, # 0.02
                'ki':                  0.0, # 0.005
                'output_limit':        4.9,
                'valve_channel':       5,
                'control_rate_hz':     500.0,
                'pressure_topic':      '/sensors/pam_pressure',
                'valve_topic':         '/actuators/pam_valve',
            }],
        ),

        # バルブ指令統合ノード
        # pos_controller (/actuators/cylinder_valves) とpam_const_pressure_controller (/actuators/pam_valve) の　[ch, volt] 指令を統合し、8ch の /actuators/valve_voltage を出力。
        Node(
            package='cylinder_exp',
            executable='mixer_node',
            name='valve_mixer_node',
            output='screen',
            parameters=[{
                'output_rate_hz': 1000.0,
                'input_topics': [
                    '/actuators/cylinder_valves',
                    '/actuators/pam_valve',
                ],
            }],
        ),
    ])
