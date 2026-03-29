# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, Float32
# import math
# import time
# from enum import Enum, auto


# class ControllerState(Enum):
#     WAITING_SENSOR = auto()
#     HOMING         = auto()
#     PRESSURIZE     = auto()
#     RUNNING        = auto()
#     STOPPED        = auto()


# class PIDController:
#     """
#     不完全微分付きPIDコントローラ。
#     アンチワインドアップ：積分値を output_limit / ki でクランプ（ki>0 のとき有効）。
#     """
#     def __init__(self, kp=0.0, ki=0.0, kd=0.0, td=0.01, output_limit=1000.0):
#         self.kp           = kp
#         self.ki           = ki
#         self.kd           = kd
#         self.td           = td
#         self.output_limit = output_limit

#         self.integral          = 0.0
#         self.prev_error        = 0.0
#         self.prev_derivative   = 0.0

#     def reset(self):
#         self.integral        = 0.0
#         self.prev_error      = 0.0
#         self.prev_derivative = 0.0

#     def update(self, target, actual, dt):
#         if dt <= 0.0:
#             return 0.0

#         error = target - actual

#         # 積分（アンチワインドアップ：出力飽和に対応した積分上限クランプ）
#         # 積分項が output_limit を超えないように上限を設定
#         # ki > 0 のとき: |ki * integral| <= output_limit  →  |integral| <= output_limit / ki
#         self.integral += error * dt
#         if self.ki > 1e-9:
#             integral_limit = self.output_limit / self.ki
#             self.integral = max(-integral_limit, min(integral_limit, self.integral))

#         # 不完全微分（ローパスフィルタ付き微分）
#         raw_derivative      = (error - self.prev_error) / dt
#         alpha               = self.td / (self.td + dt)
#         filtered_derivative = alpha * self.prev_derivative + (1.0 - alpha) * raw_derivative

#         output = (self.kp * error
#                   + self.ki * self.integral
#                   + self.kd * filtered_derivative)

#         self.prev_error      = error
#         self.prev_derivative = filtered_derivative

#         return max(-self.output_limit, min(self.output_limit, output))


# class CylinderPositionController(Node):
#     """
#     空気圧シリンダ位置制御ノード。

#     カスケード制御構成
#     ─────────────────────────────────────────────────────
#     【外側ループ】位置PID → 目標推力[N]          outer_rate_hz (推奨: 500 Hz)
#     【内側ループ】圧力PI  → バルブ電圧[V]         inner_rate_hz (推奨: 1000 Hz)
#     ─────────────────────────────────────────────────────
#     ─────────────────────────────────────────────────────
#     ステップ目標ではなく、ランプ（スロープ）軌道で中心位置まで誘導する。
#     ramp_rate_m_s [m/s] でスロープの速さを調整できる。
#     """

#     def __init__(self):
#         super().__init__('cylinder_position_controller')

#         # ── シリンダ寸法 ──────────────────────────────────────────
#         D_cyl = 0.020   # ボア径 [m]
#         d_rod = 0.010   # ロッド径 [m]
#         self.AREA_HEAD = math.pi / 4.0 * D_cyl ** 2
#         self.AREA_ROD  = math.pi / 4.0 * (D_cyl ** 2 - d_rod ** 2)

#         self.VALVE_NEUTRAL = 5.0

#         # ── パラメータ宣言 ────────────────────────────────────────

#         # ハードウェア
#         self.declare_parameter('ch_head', 0)
#         self.declare_parameter('ch_rod',  1)

#         # ループ周期
#         self.declare_parameter('outer_rate_hz', 500.0)   # 外側（位置）ループ周期 [Hz]
#         self.declare_parameter('inner_rate_hz', 1000.0)  # 内側（圧力）ループ周期 [Hz]

#         # 軌道
#         self.declare_parameter('sine_amplitude_m',   0.020)
#         self.declare_parameter('sine_freq_hz',        0.5)
#         self.declare_parameter('center_position_m',   0.060)

#         # PRESSURIZE ランプ軌道
#         self.declare_parameter('ramp_rate_m_s', 0.010)  # スロープ速度 [m/s]（例: 10 mm/s）

#         # 圧力
#         self.declare_parameter('base_pressure_kpa',   250.0)
#         self.declare_parameter('supply_pressure_kpa', 600.0)

#         # 位置ループ PID（外側）
#         self.declare_parameter('pos_kp', 2000.0)
#         self.declare_parameter('pos_ki',    0.0)
#         self.declare_parameter('pos_kd',    0.0)
#         self.declare_parameter('pos_td',    0.005)
#         self.declare_parameter('pos_output_limit', 1000.0)  # 推力上限 [N]

#         # 圧力ループ PI（内側）
#         self.declare_parameter('pres_kp',  0.02)
#         self.declare_parameter('pres_ki',  0.005)
#         self.declare_parameter('pres_kd',  0.0)
#         self.declare_parameter('pres_td',  0.01)
#         self.declare_parameter('pres_output_limit', 4.9)    # バルブ電圧加算上限 [V]
#         # ※ 中立5V ± 4.9V → 0.1〜9.9V に収まる

#         # ホーミング
#         self.declare_parameter('homing_settle_threshold', 0.0002)
#         self.declare_parameter('homing_settle_duration',  1.0)
#         self.declare_parameter('homing_startup_wait',     0.5)

#         # PRESSURIZE → RUNNING 遷移
#         self.declare_parameter('pressurize_pos_threshold',   0.002)
#         self.declare_parameter('pressurize_settle_duration', 0.5)

#         # ── 読み込み ──────────────────────────────────────────────
#         self.CH_HEAD = self.get_parameter('ch_head').value
#         self.CH_ROD  = self.get_parameter('ch_rod').value

#         outer_rate_hz = float(self.get_parameter('outer_rate_hz').value)
#         inner_rate_hz = float(self.get_parameter('inner_rate_hz').value)
#         self._outer_dt = 1.0 / outer_rate_hz
#         self._inner_dt = 1.0 / inner_rate_hz

#         # ── 状態変数 ──────────────────────────────────────────────
#         self.state       = ControllerState.WAITING_SENSOR
#         self.x_0         = 0.0
#         self.current_pos = None
#         self.current_pH  = 0.0
#         self.current_pR  = 0.0

#         # ホーミング用
#         self.homing_start_time  = None
#         self.homing_last_pos    = None
#         self.homing_settle_start = None

#         # PRESSURIZE ランプ軌道用
#         self.ramp_target        = 0.0   # ランプの現在目標値 [m]（相対座標）
#         self.pressurize_settle_start = None

#         # RUNNING 用
#         self.run_start_time = None

#         # 外側ループ用タイムスタンプ
#         self._outer_last_time = None

#         # 内側ループ用：外側が計算した目標推力を内側に渡す共有変数
#         self._target_force_N = 0.0

#         # ── PIDコントローラ ───────────────────────────────────────
#         self._init_pid()

#         # ── パブリッシャ ──────────────────────────────────────────
#         self.pub_valve = self.create_publisher(
#             Float32MultiArray, '/actuators/valve_voltage', 10)
#         self.pub_debug = self.create_publisher(
#             Float32MultiArray, '/debug/cylinder_controller', 10)
#         self.pub_target_pos = self.create_publisher(
#             Float32, '/control/target_position', 10)
#         self.pub_current_rel_pos = self.create_publisher(
#             Float32, '/control/current_rel_position', 10)

#         # ── サブスクライバ ────────────────────────────────────────
#         self.create_subscription(
#             Float32, '/sensors/cylinder_position', self._cb_pos, 10)
#         self.create_subscription(
#             Float32, '/sensors/head_pressure',     self._cb_ph,  10)
#         self.create_subscription(
#             Float32, '/sensors/rod_pressure',      self._cb_pr,  10)

#         # ── タイマ（外側・内側を分離） ────────────────────────────
#         self._outer_timer = self.create_timer(self._outer_dt, self._outer_loop)
#         self._inner_timer = self.create_timer(self._inner_dt, self._inner_loop)

#         self.get_logger().info(
#             f"Controller initialized. "
#             f"outer={outer_rate_hz:.0f}Hz / inner={inner_rate_hz:.0f}Hz. "
#             f"Waiting for sensors..."
#         )

#     # ── PID 初期化ヘルパ ─────────────────────────────────────────
#     def _init_pid(self):
#         pos_limit  = float(self.get_parameter('pos_output_limit').value)
#         pres_limit = float(self.get_parameter('pres_output_limit').value)
#         self.pid_pos = PIDController(output_limit=pos_limit)
#         self.pid_pH  = PIDController(output_limit=pres_limit)
#         self.pid_pR  = PIDController(output_limit=pres_limit)
#         self._update_gains()

#     def _update_gains(self):
#         self.pid_pos.kp = float(self.get_parameter('pos_kp').value)
#         self.pid_pos.ki = float(self.get_parameter('pos_ki').value)
#         self.pid_pos.kd = float(self.get_parameter('pos_kd').value)
#         self.pid_pos.td = float(self.get_parameter('pos_td').value)
#         self.pid_pos.output_limit = float(self.get_parameter('pos_output_limit').value)

#         pres_limit = float(self.get_parameter('pres_output_limit').value)
#         for pid in [self.pid_pH, self.pid_pR]:
#             pid.kp           = float(self.get_parameter('pres_kp').value)
#             pid.ki           = float(self.get_parameter('pres_ki').value)
#             pid.kd           = float(self.get_parameter('pres_kd').value)
#             pid.td           = float(self.get_parameter('pres_td').value)
#             pid.output_limit = pres_limit

#     # ── センサコールバック ────────────────────────────────────────
#     def _cb_pos(self, msg): self.current_pos = float(msg.data)
#     def _cb_ph(self, msg):  self.current_pH  = float(msg.data)
#     def _cb_pr(self, msg):  self.current_pR  = float(msg.data)

#     # ── バルブ出力ヘルパ ─────────────────────────────────────────
#     def _send_valve(self, volt_h, volt_r):
#         volt_h = max(0.0, min(10.0, volt_h))
#         volt_r = max(0.0, min(10.0, volt_r))
#         msg = Float32MultiArray()
#         msg.data = [self.VALVE_NEUTRAL] * 8
#         msg.data[self.CH_HEAD] = volt_h
#         msg.data[self.CH_ROD]  = volt_r
#         self.pub_valve.publish(msg)

#     def _send_all_neutral(self):
#         self._send_valve(self.VALVE_NEUTRAL, self.VALVE_NEUTRAL)

#     def _get_relative_pos(self):
#         return self.current_pos - self.x_0

#     # ────────────────────────────────────────────────────────────
#     # 外側ループ（位置 PID）  ← outer_rate_hz で実行
#     # ────────────────────────────────────────────────────────────
#     def _outer_loop(self):
#         now = time.time()

#         if self.state == ControllerState.WAITING_SENSOR:
#             self._state_waiting_sensor()
#             return

#         if self.state == ControllerState.HOMING:
#             self._state_homing(now)
#             return

#         # PRESSURIZE / RUNNING は dt が必要なので初回はスキップ
#         if self._outer_last_time is None:
#             self._outer_last_time = now
#             return

#         dt = now - self._outer_last_time
#         if dt <= 0.0:
#             return
#         self._outer_last_time = now
#         self._update_gains()

#         if self.state == ControllerState.PRESSURIZE:
#             self._state_pressurize(now, dt)
#         elif self.state == ControllerState.RUNNING:
#             self._state_running(now, dt)
#         else:
#             # STOPPED など
#             self._target_force_N = 0.0
#             self._send_all_neutral()

#     # ────────────────────────────────────────────────────────────
#     # 内側ループ（圧力 PI → バルブ電圧）  ← inner_rate_hz で実行
#     # ────────────────────────────────────────────────────────────
#     def _inner_loop(self):
#         # PRESSURIZE / RUNNING 以外は外側ループで制御するため内側は何もしない
#         if self.state not in (ControllerState.PRESSURIZE, ControllerState.RUNNING):
#             return

#         dt = self._inner_dt  # 固定 dt（タイマ周期そのまま使用）
#         self._apply_force_to_valves(self._target_force_N, dt)

#     # ────────────────────────────────────────────────────────────
#     # 各状態の処理
#     # ────────────────────────────────────────────────────────────
#     def _state_waiting_sensor(self):
#         self._send_all_neutral()
#         if self.current_pos is not None:
#             self.get_logger().info("Sensors connected. Starting homing...")
#             self.state              = ControllerState.HOMING
#             self.homing_start_time  = time.time()
#             self.homing_last_pos    = self.current_pos
#             self.homing_settle_start = None

#     def _state_homing(self, now):
#         settle_thresh = float(self.get_parameter('homing_settle_threshold').value)
#         settle_dur    = float(self.get_parameter('homing_settle_duration').value)
#         startup_wait  = float(self.get_parameter('homing_startup_wait').value)

#         # ホーミング中はロッド側全開（縮み方向）
#         self._send_valve(0.0, 10.0)

#         if now - self.homing_start_time < startup_wait:
#             self.homing_last_pos = self.current_pos
#             return

#         pos_change = abs(self.current_pos - self.homing_last_pos)

#         if pos_change < settle_thresh:
#             if self.homing_settle_start is None:
#                 self.homing_settle_start = now
#             elif now - self.homing_settle_start > settle_dur:
#                 # 原点確定
#                 self.x_0 = self.current_pos
#                 self.get_logger().info(
#                     f"Homing complete. Origin = {self.x_0:.4f} m")

#                 # ランプ軌道の初期値を現在の相対位置（= 0）に設定
#                 self.ramp_target          = 0.0
#                 self.pressurize_settle_start = None

#                 # PID リセット
#                 self.pid_pos.reset()
#                 self.pid_pH.reset()
#                 self.pid_pR.reset()
#                 self._target_force_N  = 0.0
#                 self._outer_last_time = time.time()

#                 self.state = ControllerState.PRESSURIZE
#                 return
#         else:
#             self.homing_settle_start = None

#         self.homing_last_pos = self.current_pos

#     def _state_pressurize(self, now, dt):
#         """
#         ランプ軌道で中心位置まで移動。
#         ステップ目標を廃止し、ramp_target を毎ループ ramp_rate_m_s × dt だけ
#         center に近づけることで急峻な誤差・振動を防ぐ。
#         """
#         center     = float(self.get_parameter('center_position_m').value)
#         ramp_rate  = float(self.get_parameter('ramp_rate_m_s').value)
#         pos_thresh = float(self.get_parameter('pressurize_pos_threshold').value)
#         settle_dur = float(self.get_parameter('pressurize_settle_duration').value)

#         # ── ランプ軌道の更新 ──────────────────────────────────────
#         step = ramp_rate * dt
#         if self.ramp_target < center:
#             self.ramp_target = min(self.ramp_target + step, center)
#         else:
#             self.ramp_target = max(self.ramp_target - step, center)

#         x_rel = self._get_relative_pos()

#         # 外側 PID：ランプ目標を追従
#         self._target_force_N = self.pid_pos.update(self.ramp_target, x_rel, dt)

#         # トピック出力
#         self.pub_target_pos.publish(Float32(data=float(self.ramp_target)))
#         self.pub_current_rel_pos.publish(Float32(data=float(x_rel)))

#         # ── 安定判定（実際の中心への収束を確認） ─────────────────
#         if abs(x_rel - center) < pos_thresh:
#             if self.pressurize_settle_start is None:
#                 self.pressurize_settle_start = now
#             elif now - self.pressurize_settle_start > settle_dur:
#                 self.get_logger().info(
#                     f"Center reached ({x_rel * 1000:.1f} mm). Starting sine wave.")
#                 # PID リセット & RUNNING へ遷移
#                 self.pid_pos.reset()
#                 self.pid_pH.reset()
#                 self.pid_pR.reset()
#                 self._target_force_N  = 0.0
#                 self._outer_last_time = time.time()
#                 self.run_start_time   = time.time()
#                 self.state            = ControllerState.RUNNING
#         else:
#             self.pressurize_settle_start = None

#     def _state_running(self, now, dt):
#         """
#         正弦波追従制御。
#         """
#         amp    = float(self.get_parameter('sine_amplitude_m').value)
#         freq   = float(self.get_parameter('sine_freq_hz').value)
#         center = float(self.get_parameter('center_position_m').value)

#         elapsed = now - self.run_start_time
#         x_ref   = center + amp * math.sin(2.0 * math.pi * freq * elapsed)

#         x_rel = self._get_relative_pos()

#         # 外側 PID → 目標推力（内側ループへ渡す）
#         self._target_force_N = self.pid_pos.update(x_ref, x_rel, dt)

#         # トピック出力
#         self.pub_target_pos.publish(Float32(data=float(x_ref)))
#         self.pub_current_rel_pos.publish(Float32(data=float(x_rel)))

#     # ────────────────────────────────────────────────────────────
#     # 推力 → 圧力目標 → 圧力 PI → バルブ電圧
#     # （内側ループから呼ばれる）
#     # ────────────────────────────────────────────────────────────
#     def _apply_force_to_valves(self, target_force_N, dt):
#         """
#         片側加圧方式：
#         両室ともベース圧を維持しつつ、推力方向の室だけ加圧する。
#         """
#         base_kpa = float(self.get_parameter('base_pressure_kpa').value)
#         max_kpa  = float(self.get_parameter('supply_pressure_kpa').value)

#         # ① 推力 → 目標圧力（フィードフォワード的変換）
#         target_pH = base_kpa
#         target_pR = base_kpa
#         if target_force_N > 0.0:
#             target_pH += target_force_N / self.AREA_HEAD / 1000.0
#         elif target_force_N < 0.0:
#             target_pR += (-target_force_N) / self.AREA_ROD / 1000.0

#         target_pH = max(0.0, min(max_kpa, target_pH))
#         target_pR = max(0.0, min(max_kpa, target_pR))

#         # ② 圧力 PI → バルブ加算電圧
#         uH = self.pid_pH.update(target_pH, self.current_pH, dt)
#         uR = self.pid_pR.update(target_pR, self.current_pR, dt)

#         self._send_valve(self.VALVE_NEUTRAL + uH, self.VALVE_NEUTRAL + uR)

#         # デバッグ出力
#         msg = Float32MultiArray()
#         msg.data = [
#             float(target_pH),       # [0] 目標ヘッド圧 [kPa]
#             float(self.current_pH), # [1] 現在ヘッド圧 [kPa]
#             float(target_pR),       # [2] 目標ロッド圧 [kPa]
#             float(self.current_pR), # [3] 現在ロッド圧 [kPa]
#             float(uH),              # [4] ヘッド側バルブ加算電圧 [V]
#             float(uR),              # [5] ロッド側バルブ加算電圧 [V]
#             float(target_force_N),  # [6] 目標推力 [N]
#         ]
#         self.pub_debug.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = CylinderPositionController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down...")
#     finally:
#         node._send_all_neutral()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import math
import time
from enum import Enum, auto


class ControllerState(Enum):
    WAITING_SENSOR = auto()
    HOMING         = auto()
    PRESSURIZE     = auto()
    RUNNING        = auto()
    STOPPED        = auto()


class PIDController:
    """
    不完全微分付きPIDコントローラ。
    アンチワインドアップ：
        積分値を output_limit / ki でクランプ（ki>0 のとき有効）
    """
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, td=0.01, output_limit=1000.0):
        self.kp           = kp
        self.ki           = ki
        self.kd           = kd
        self.td           = td
        self.output_limit = output_limit

        self.integral        = 0.0
        self.prev_error      = 0.0
        self.prev_derivative = 0.0

    def reset(self):
        self.integral        = 0.0
        self.prev_error      = 0.0
        self.prev_derivative = 0.0

    def update(self, target, actual, dt):
        if dt <= 0.0:
            return 0.0

        error = target - actual

        # 積分（アンチワインドアップ）
        self.integral += error * dt
        if self.ki > 1e-9:
            integral_limit = self.output_limit / self.ki
            self.integral = max(-integral_limit, min(integral_limit, self.integral))

        # 不完全微分（ローパスフィルタ付き微分）
        raw_derivative      = (error - self.prev_error) / dt
        alpha               = self.td / (self.td + dt)
        filtered_derivative = alpha * self.prev_derivative + (1.0 - alpha) * raw_derivative

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * filtered_derivative
        )

        self.prev_error      = error
        self.prev_derivative = filtered_derivative

        return max(-self.output_limit, min(self.output_limit, output))


class CylinderPositionController(Node):
    """
    空気圧シリンダ位置制御ノード。

    カスケード制御構成
    ─────────────────────────────────────────────────────
    【外側ループ】位置PID → 目標推力[N]          outer_rate_hz
    【内側ループ】圧力PI  → バルブ電圧[V]         inner_rate_hz
    ─────────────────────────────────────────────────────

    オプション：
    ロードセル値を外乱補償項として加算可能
        F_target = F_pos + loadcell_ff_gain * F_loadcell

    ※ ロードセル値はセンサ側ですでにフィルタ済みの前提
    ※ ロードセル正方向と位置正方向は同じ前提
    ※ 補償の向きは loadcell_ff_gain の符号で調整する
    """

    def __init__(self):
        super().__init__('cylinder_position_controller')

        # ── シリンダ寸法 ──────────────────────────────────────────
        D_cyl = 0.020   # ボア径 [m]
        d_rod = 0.010   # ロッド径 [m]
        self.AREA_HEAD = math.pi / 4.0 * D_cyl ** 2
        self.AREA_ROD  = math.pi / 4.0 * (D_cyl ** 2 - d_rod ** 2)

        self.VALVE_NEUTRAL = 5.0

        # ── パラメータ宣言 ────────────────────────────────────────

        # ハードウェア
        self.declare_parameter('ch_head', 0)
        self.declare_parameter('ch_rod',  1)

        # ループ周期
        self.declare_parameter('outer_rate_hz', 500.0)
        self.declare_parameter('inner_rate_hz', 1000.0)

        # 軌道
        self.declare_parameter('sine_amplitude_m', 0.020)
        self.declare_parameter('sine_freq_hz', 0.5)
        self.declare_parameter('center_position_m', 0.060)

        # PRESSURIZE ランプ軌道
        self.declare_parameter('ramp_rate_m_s', 0.010)

        # 圧力
        self.declare_parameter('base_pressure_kpa', 250.0)
        self.declare_parameter('supply_pressure_kpa', 600.0)

        # 位置ループ PID（外側）
        self.declare_parameter('pos_kp', 2000.0)
        self.declare_parameter('pos_ki', 0.0)
        self.declare_parameter('pos_kd', 0.0)
        self.declare_parameter('pos_td', 0.005)
        self.declare_parameter('pos_output_limit', 1000.0)  # 推力上限 [N]

        # 圧力ループ PI（内側）
        self.declare_parameter('pres_kp', 0.02)
        self.declare_parameter('pres_ki', 0.005)
        self.declare_parameter('pres_kd', 0.0)
        self.declare_parameter('pres_td', 0.01)
        self.declare_parameter('pres_output_limit', 4.9)    # バルブ電圧加算上限 [V]

        # ホーミング
        self.declare_parameter('homing_settle_threshold', 0.0002)
        self.declare_parameter('homing_settle_duration', 1.0)
        self.declare_parameter('homing_startup_wait', 0.5)

        # PRESSURIZE → RUNNING 遷移
        self.declare_parameter('pressurize_pos_threshold', 0.002)
        self.declare_parameter('pressurize_settle_duration', 0.5)

        # ロードセル補償
        self.declare_parameter('use_loadcell_compensation', False)
        self.declare_parameter('loadcell_ff_gain', 0.3)      # 正負どちらでも可
        self.declare_parameter('loadcell_timeout_s', 0.2)
        self.declare_parameter('loadcell_use_in_pressurize', False)

        # ── パラメータ読み込み ────────────────────────────────────
        self.CH_HEAD = int(self.get_parameter('ch_head').value)
        self.CH_ROD  = int(self.get_parameter('ch_rod').value)

        outer_rate_hz = float(self.get_parameter('outer_rate_hz').value)
        inner_rate_hz = float(self.get_parameter('inner_rate_hz').value)
        self._outer_dt = 1.0 / outer_rate_hz
        self._inner_dt = 1.0 / inner_rate_hz

        # ── 状態変数 ──────────────────────────────────────────────
        self.state       = ControllerState.WAITING_SENSOR
        self.x_0         = 0.0
        self.current_pos = None
        self.current_pH  = 0.0
        self.current_pR  = 0.0

        # ロードセル
        self.current_loadcell = None
        self._loadcell_last_msg_time = None
        self._loadcell_timeout_warned = False

        # デバッグ保持
        self._last_pos_force_N = 0.0
        self._last_loadcell_comp_N = 0.0

        # ホーミング用
        self.homing_start_time    = None
        self.homing_last_pos      = None
        self.homing_settle_start  = None

        # PRESSURIZE ランプ軌道用
        self.ramp_target = 0.0
        self.pressurize_settle_start = None

        # RUNNING 用
        self.run_start_time = None

        # 外側ループ用タイムスタンプ
        self._outer_last_time = None

        # 内側ループへ渡す目標推力
        self._target_force_N = 0.0

        # ── PIDコントローラ ───────────────────────────────────────
        self._init_pid()

        # ── パブリッシャ ──────────────────────────────────────────
        self.pub_valve = self.create_publisher(
            Float32MultiArray, '/actuators/valve_voltage', 10)
        self.pub_debug = self.create_publisher(
            Float32MultiArray, '/debug/cylinder_controller', 10)
        self.pub_target_pos = self.create_publisher(
            Float32, '/control/target_position', 10)
        self.pub_current_rel_pos = self.create_publisher(
            Float32, '/control/current_rel_position', 10)

        # ── サブスクライバ ────────────────────────────────────────
        self.create_subscription(
            Float32, '/sensors/cylinder_position', self._cb_pos, 10)
        self.create_subscription(
            Float32, '/sensors/head_pressure', self._cb_ph, 10)
        self.create_subscription(
            Float32, '/sensors/rod_pressure', self._cb_pr, 10)
        self.create_subscription(
            Float32, '/sensors/loadcell_force', self._cb_loadcell, 10)

        # ── タイマ ────────────────────────────────────────────────
        self._outer_timer = self.create_timer(self._outer_dt, self._outer_loop)
        self._inner_timer = self.create_timer(self._inner_dt, self._inner_loop)

        self.get_logger().info(
            f"Controller initialized. "
            f"outer={outer_rate_hz:.0f}Hz / inner={inner_rate_hz:.0f}Hz. "
            f"Waiting for sensors..."
        )

    # ── PID 初期化ヘルパ ─────────────────────────────────────────
    def _init_pid(self):
        pos_limit  = float(self.get_parameter('pos_output_limit').value)
        pres_limit = float(self.get_parameter('pres_output_limit').value)

        self.pid_pos = PIDController(output_limit=pos_limit)
        self.pid_pH  = PIDController(output_limit=pres_limit)
        self.pid_pR  = PIDController(output_limit=pres_limit)

        self._update_gains()

    def _update_gains(self):
        self.pid_pos.kp = float(self.get_parameter('pos_kp').value)
        self.pid_pos.ki = float(self.get_parameter('pos_ki').value)
        self.pid_pos.kd = float(self.get_parameter('pos_kd').value)
        self.pid_pos.td = float(self.get_parameter('pos_td').value)
        self.pid_pos.output_limit = float(self.get_parameter('pos_output_limit').value)

        pres_limit = float(self.get_parameter('pres_output_limit').value)
        for pid in [self.pid_pH, self.pid_pR]:
            pid.kp = float(self.get_parameter('pres_kp').value)
            pid.ki = float(self.get_parameter('pres_ki').value)
            pid.kd = float(self.get_parameter('pres_kd').value)
            pid.td = float(self.get_parameter('pres_td').value)
            pid.output_limit = pres_limit

    # ── センサコールバック ────────────────────────────────────────
    def _cb_pos(self, msg):
        self.current_pos = float(msg.data)

    def _cb_ph(self, msg):
        self.current_pH = float(msg.data)

    def _cb_pr(self, msg):
        self.current_pR = float(msg.data)

    def _cb_loadcell(self, msg):
        self.current_loadcell = float(msg.data)
        self._loadcell_last_msg_time = time.monotonic()

    # ── ユーティリティ ───────────────────────────────────────────
    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def _send_valve(self, volt_h, volt_r):
        volt_h = self._clamp(volt_h, 0.0, 10.0)
        volt_r = self._clamp(volt_r, 0.0, 10.0)

        msg = Float32MultiArray()
        msg.data = [self.VALVE_NEUTRAL] * 8
        msg.data[self.CH_HEAD] = volt_h
        msg.data[self.CH_ROD]  = volt_r
        self.pub_valve.publish(msg)

    def _send_all_neutral(self):
        self._send_valve(self.VALVE_NEUTRAL, self.VALVE_NEUTRAL)

    def _get_relative_pos(self):
        return self.current_pos - self.x_0

    def _compose_target_force(self, pos_force_N, now, allow_loadcell):
        """
        位置PIDの出力 pos_force_N に、必要ならロードセル補償を加える。
        ロードセル値はセンサ側で既にフィルタ済みのものをそのまま使う。
        補償の向きは loadcell_ff_gain の符号で調整する。
        """
        self._last_pos_force_N = pos_force_N
        self._last_loadcell_comp_N = 0.0

        use_lc = bool(self.get_parameter('use_loadcell_compensation').value) and allow_loadcell
        limit = float(self.get_parameter('pos_output_limit').value)

        if not use_lc:
            return self._clamp(pos_force_N, -limit, limit)

        timeout_s = float(self.get_parameter('loadcell_timeout_s').value)
        if (self._loadcell_last_msg_time is None) or ((now - self._loadcell_last_msg_time) > timeout_s):
            if not self._loadcell_timeout_warned:
                self.get_logger().warn(
                    "Loadcell compensation enabled, but loadcell data is timed out. "
                    "Compensation disabled temporarily."
                )
                self._loadcell_timeout_warned = True
            return self._clamp(pos_force_N, -limit, limit)

        if self._loadcell_timeout_warned:
            self.get_logger().info("Loadcell signal recovered. Compensation resumed.")
            self._loadcell_timeout_warned = False

        loadcell_force = 0.0 if self.current_loadcell is None else float(self.current_loadcell)
        loadcell_gain  = float(self.get_parameter('loadcell_ff_gain').value)

        comp_force = loadcell_gain * loadcell_force
        self._last_loadcell_comp_N = comp_force

        return self._clamp(pos_force_N + comp_force, -limit, limit)

    # ────────────────────────────────────────────────────────────
    # 外側ループ（位置 PID）
    # ────────────────────────────────────────────────────────────
    def _outer_loop(self):
        now = time.monotonic()

        if self.state == ControllerState.WAITING_SENSOR:
            self._state_waiting_sensor(now)
            return

        if self.state == ControllerState.HOMING:
            self._state_homing(now)
            return

        if self._outer_last_time is None:
            self._outer_last_time = now
            return

        dt = now - self._outer_last_time
        if dt <= 0.0:
            return

        self._outer_last_time = now
        self._update_gains()

        if self.state == ControllerState.PRESSURIZE:
            self._state_pressurize(now, dt)
        elif self.state == ControllerState.RUNNING:
            self._state_running(now, dt)
        else:
            self._target_force_N = 0.0
            self._send_all_neutral()

    # ────────────────────────────────────────────────────────────
    # 内側ループ（圧力 PI → バルブ電圧）
    # ────────────────────────────────────────────────────────────
    def _inner_loop(self):
        if self.state not in (ControllerState.PRESSURIZE, ControllerState.RUNNING):
            return

        dt = self._inner_dt
        self._apply_force_to_valves(self._target_force_N, dt)

    # ────────────────────────────────────────────────────────────
    # 各状態の処理
    # ────────────────────────────────────────────────────────────
    def _state_waiting_sensor(self, now):
        self._send_all_neutral()

        if self.current_pos is not None:
            self.get_logger().info("Sensors connected. Starting homing...")
            self.state = ControllerState.HOMING
            self.homing_start_time = now
            self.homing_last_pos = self.current_pos
            self.homing_settle_start = None

    def _state_homing(self, now):
        settle_thresh = float(self.get_parameter('homing_settle_threshold').value)
        settle_dur    = float(self.get_parameter('homing_settle_duration').value)
        startup_wait  = float(self.get_parameter('homing_startup_wait').value)

        # ホーミング中はロッド側全開（縮み方向）
        self._send_valve(0.0, 10.0)

        if now - self.homing_start_time < startup_wait:
            self.homing_last_pos = self.current_pos
            return

        pos_change = abs(self.current_pos - self.homing_last_pos)

        if pos_change < settle_thresh:
            if self.homing_settle_start is None:
                self.homing_settle_start = now
            elif now - self.homing_settle_start > settle_dur:
                self.x_0 = self.current_pos
                self.get_logger().info(f"Homing complete. Origin = {self.x_0:.4f} m")

                self.ramp_target = 0.0
                self.pressurize_settle_start = None

                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self._target_force_N  = 0.0
                self._outer_last_time = time.monotonic()

                self.state = ControllerState.PRESSURIZE
                return
        else:
            self.homing_settle_start = None

        self.homing_last_pos = self.current_pos

    def _state_pressurize(self, now, dt):
        """
        ランプ軌道で中心位置まで移動。
        """
        center     = float(self.get_parameter('center_position_m').value)
        ramp_rate  = float(self.get_parameter('ramp_rate_m_s').value)
        pos_thresh = float(self.get_parameter('pressurize_pos_threshold').value)
        settle_dur = float(self.get_parameter('pressurize_settle_duration').value)

        # ランプ軌道更新
        step = ramp_rate * dt
        if self.ramp_target < center:
            self.ramp_target = min(self.ramp_target + step, center)
        else:
            self.ramp_target = max(self.ramp_target - step, center)

        x_rel = self._get_relative_pos()

        # 外側 PID
        pos_force = self.pid_pos.update(self.ramp_target, x_rel, dt)

        allow_loadcell = bool(self.get_parameter('loadcell_use_in_pressurize').value)
        self._target_force_N = self._compose_target_force(pos_force, now, allow_loadcell)

        self.pub_target_pos.publish(Float32(data=float(self.ramp_target)))
        self.pub_current_rel_pos.publish(Float32(data=float(x_rel)))

        # 安定判定
        if abs(x_rel - center) < pos_thresh:
            if self.pressurize_settle_start is None:
                self.pressurize_settle_start = now
            elif now - self.pressurize_settle_start > settle_dur:
                self.get_logger().info(
                    f"Center reached ({x_rel * 1000.0:.1f} mm). Starting sine wave."
                )

                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self._target_force_N  = 0.0
                self._outer_last_time = time.monotonic()
                self.run_start_time   = time.monotonic()
                self.state            = ControllerState.RUNNING
        else:
            self.pressurize_settle_start = None

    def _state_running(self, now, dt):
        """
        正弦波追従制御
        """
        amp    = float(self.get_parameter('sine_amplitude_m').value)
        freq   = float(self.get_parameter('sine_freq_hz').value)
        center = float(self.get_parameter('center_position_m').value)

        elapsed = now - self.run_start_time
        x_ref   = center + amp * math.sin(2.0 * math.pi * freq * elapsed)

        x_rel = self._get_relative_pos()

        pos_force = self.pid_pos.update(x_ref, x_rel, dt)
        self._target_force_N = self._compose_target_force(
            pos_force, now, allow_loadcell=True
        )

        self.pub_target_pos.publish(Float32(data=float(x_ref)))
        self.pub_current_rel_pos.publish(Float32(data=float(x_rel)))

    # ────────────────────────────────────────────────────────────
    # 推力 → 圧力目標 → 圧力 PI → バルブ電圧
    # ────────────────────────────────────────────────────────────
    def _apply_force_to_valves(self, target_force_N, dt):
        """
        片側加圧方式：
        両室ともベース圧を維持しつつ、推力方向の室だけ加圧する。
        """
        base_kpa = float(self.get_parameter('base_pressure_kpa').value)
        max_kpa  = float(self.get_parameter('supply_pressure_kpa').value)

        # 推力 → 圧力目標
        target_pH = base_kpa
        target_pR = base_kpa

        if target_force_N > 0.0:
            target_pH += target_force_N / self.AREA_HEAD / 1000.0
        elif target_force_N < 0.0:
            target_pR += (-target_force_N) / self.AREA_ROD / 1000.0

        target_pH = self._clamp(target_pH, 0.0, max_kpa)
        target_pR = self._clamp(target_pR, 0.0, max_kpa)

        # 圧力 PI → バルブ加算電圧
        uH = self.pid_pH.update(target_pH, self.current_pH, dt)
        uR = self.pid_pR.update(target_pR, self.current_pR, dt)

        self._send_valve(self.VALVE_NEUTRAL + uH, self.VALVE_NEUTRAL + uR)

        # デバッグ出力
        msg = Float32MultiArray()
        msg.data = [
            float(target_pH),                                          # [0] 目標ヘッド圧 [kPa]
            float(self.current_pH),                                    # [1] 現在ヘッド圧 [kPa]
            float(target_pR),                                          # [2] 目標ロッド圧 [kPa]
            float(self.current_pR),                                    # [3] 現在ロッド圧 [kPa]
            float(uH),                                                 # [4] ヘッド側バルブ加算電圧 [V]
            float(uR),                                                 # [5] ロッド側バルブ加算電圧 [V]
            float(target_force_N),                                     # [6] 最終目標推力 [N]
            float(0.0 if self.current_loadcell is None else self.current_loadcell),  # [7] ロードセル値 [N]
            float(self._last_pos_force_N),                             # [8] 位置PID由来の推力 [N]
            float(self._last_loadcell_comp_N),                         # [9] ロードセル補償項 [N]
        ]
        self.pub_debug.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CylinderPositionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node._send_all_neutral()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
