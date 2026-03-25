from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── 1. AI ボードノード ────────────────────────────────────
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen',
        ),

        # ── 2. AO ボードノード ────────────────────────────────────
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='ao1608llpe_node',
            output='screen',
        ),

        # ── 3. カウンタボード（エンコーダ）ノード ─────────────────
        Node(
            package='control_box',
            executable='cnt3204mtlpe_test',
            name='cnt3204mtlpe_node',
            output='screen',
        ),

        # ── 4. 信号変換・フィルタノード ───────────────────────────
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

        # ── 5. シリンダ位置制御ノード ─────────────────────────────
        Node(
            package='cylinder_exp',
            executable='pos_controller',
            name='cylinder_position_controller_node',
            output='screen',
            parameters=[{

                # ハードウェア
                'ch_head': 0,   # AO ボードのヘッド側チャンネル番号
                'ch_rod':  1,   # AO ボードのロッド側チャンネル番号

                # ループ周期
                # 外側（位置）ループ。500 Hz 推奨。
                # Python + rclpy で安定して回せる上限を実測して決定すること。
                'outer_rate_hz': 500.0,

                # 内側（圧力）ループ。外側の 2〜5 倍推奨。
                # Python + rclpy で安定して回せる上限を実測して決定すること。
                'inner_rate_hz': 1000.0,

                # 正弦波軌道
                'sine_amplitude_m':  0.060,   # 振幅 [m]  （4 mm）
                'sine_freq_hz':      0.8,     # 周波数 [Hz]
                'center_position_m': 0.080,   # 中心位置 [m] （5 mm）

                # PRESSURIZE ランプ軌道
                # 原点から center_position_m まで近づく速度 [m/s]。
                # 小さいほどゆっくり・振動しにくい。大きいほど素早く移動。
                # 例: 0.005 → 5 mm/s ≒ center=5 mm に 1 秒で到達
                'ramp_rate_m_s': 0.004,

                # 圧力
                'base_pressure_kpa':   50.0,  # 両室のベース圧力 [kPa]
                'supply_pressure_kpa': 600.0,  # 圧力指令の上限（供給圧以下に設定）[kPa]

                # 位置ループPID(外側)
                'pos_kp': 600.0,   # 比例ゲイン [N/m]
                'pos_ki': 200.0,   # 積分ゲイン [N/(m·s)]
                'pos_kd': 0.0,   # 微分ゲイン [N·s/m]
                'pos_td': 0.05,  # 微分フィルタ時定数 [s]
                'pos_output_limit': 1000.0,  # 推力指令の上限 [N]

                # 圧力ループ PI（内側）
                'pres_kp':  0.019,   # 比例ゲイン [V/kPa]
                'pres_ki':  0.01,   # 積分ゲイン [V/(kPa·s)]
                'pres_kd':  0.0,    # 微分ゲイン（通常 0）
                'pres_td':  0.01,   # 微分フィルタ時定数 [s]
                # バルブ加算電圧上限 [V]。中立 5V ± この値 → 0.1〜9.9V に収まるよう 4.9 推奨。
                'pres_output_limit': 4.9,

                # ホーミング
                'homing_settle_threshold': 0.0002,  # 位置変化がこれ以下で安定とみなす [m]
                'homing_settle_duration':  1.0,     # 安定判定の継続時間 [s]
                'homing_startup_wait':     0.5,     # 起動直後の待機時間 [s]

                # PRESSURIZE → RUNNING 遷移
                'pressurize_pos_threshold':   0.002,  # 中心との誤差がこれ以下で収束とみなす [m]
                'pressurize_settle_duration': 0.5,    # 収束判定の継続時間 [s]
            }],
        ),
    ])
