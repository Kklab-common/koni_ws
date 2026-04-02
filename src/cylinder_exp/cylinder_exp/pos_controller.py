import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import math
import time
from enum import Enum, auto


class ControllerState(Enum):
    WAITING_SENSOR = auto()
    HOMING         = auto()
    RUNNING        = auto()
    STOPPED        = auto()


class PIDController:
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

        self.integral += error * dt
        if self.ki > 1e-9:
            integral_limit = self.output_limit / self.ki
            self.integral = max(-integral_limit, min(integral_limit, self.integral))

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
    空気圧シリンダ位置制御ノード（PAM接続版）。

    ホーミング:
        ヘッド側 homing_head_voltage / ロッド側 0V
        → ロッド伸び切り位置 = x_0 = 正弦波の最小端

    RUNNING:
        x_ref_rel = A * (1 - cos(ωt))
        x_0(最小端) → x_0+2A(最大端) を往復

    力の符号規約:
        正 = 押し出し方向（右, ロッド伸び, PAM縮み）
        負 = 引っ込み方向（左, ロッド縮み, PAM伸び）

    ロードセル符号規約:
        PAMに引っ張られている時 → 負
        F_target = F_pid - gain * F_loadcell
        （F_loadcell が負 → 補償力は正 → 押し出し方向に加算 → 正しい）
    """

    def __init__(self):
        super().__init__('cylinder_position_controller')

        # ── シリンダ寸法 ──────────────────────────────────────────
        D_cyl = 0.020
        d_rod = 0.010
        self.AREA_HEAD = math.pi / 4.0 * D_cyl ** 2
        self.AREA_ROD  = math.pi / 4.0 * (D_cyl ** 2 - d_rod ** 2)

        self.VALVE_NEUTRAL = 5.0

        # ── パラメータ宣言 ────────────────────────────────────────

        self.declare_parameter('ch_head', 0)
        self.declare_parameter('ch_rod',  1)

        self.declare_parameter('outer_rate_hz', 500.0)
        self.declare_parameter('inner_rate_hz', 1000.0)

        self.declare_parameter('sine_amplitude_m', 0.020)
        self.declare_parameter('sine_freq_hz', 0.5)
        self.declare_parameter('sine_amp_ramp_rate_m_s', 0.010)

        self.declare_parameter('base_pressure_kpa', 250.0)
        self.declare_parameter('supply_pressure_kpa', 600.0)

        self.declare_parameter('pos_kp', 2000.0)
        self.declare_parameter('pos_ki', 0.0)
        self.declare_parameter('pos_kd', 0.0)
        self.declare_parameter('pos_td', 0.005)
        self.declare_parameter('pos_output_limit', 1000.0)

        self.declare_parameter('gain_ramp_start_ratio', 0.1)
        self.declare_parameter('gain_ramp_duration_s', 3.0)

        self.declare_parameter('pres_kp', 0.02)
        self.declare_parameter('pres_ki', 0.005)
        self.declare_parameter('pres_kd', 0.0)
        self.declare_parameter('pres_td', 0.01)
        self.declare_parameter('pres_output_limit', 4.9)

        self.declare_parameter('homing_settle_threshold', 0.0002)
        self.declare_parameter('homing_settle_duration', 1.0)
        self.declare_parameter('homing_startup_wait', 0.5)
        self.declare_parameter('homing_head_voltage', 6.0)

        self.declare_parameter('use_loadcell_compensation', False)
        self.declare_parameter('loadcell_ff_gain', 0.3)
        self.declare_parameter('loadcell_timeout_s', 0.2)

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

        self.current_loadcell = None
        self._loadcell_last_msg_time = None
        self._loadcell_timeout_warned = False

        self._last_pos_force_N = 0.0
        self._last_loadcell_comp_N = 0.0

        self.homing_start_time   = None
        self.homing_last_pos     = None
        self.homing_settle_start = None

        self.current_sine_amp = 0.0
        self.run_start_time   = None

        self._current_gain_ratio = 0.0

        self._outer_last_time = None
        self._target_force_N  = 0.0

        # 内側ループ出力保持（デバッグ用）
        self._last_target_pH = 0.0
        self._last_target_pR = 0.0
        self._last_uH = 0.0
        self._last_uR = 0.0

        # ── PIDコントローラ ───────────────────────────────────────
        self._init_pid()

        # ── パブリッシャ（個別名前付きトピック） ──────────────────
        self.pub_valve = self.create_publisher(
            Float32MultiArray, '/actuators/valve_voltage', 10)

        # 位置系
        self.pub_target_pos = self.create_publisher(
            Float32, '/debug/target_position_m', 10)
        self.pub_current_rel_pos = self.create_publisher(
            Float32, '/debug/current_rel_position_m', 10)
        self.pub_position_error = self.create_publisher(
            Float32, '/debug/position_error_m', 10)

        # 力系
        self.pub_target_force = self.create_publisher(
            Float32, '/debug/target_force_N', 10)
        self.pub_pid_force = self.create_publisher(
            Float32, '/debug/pid_force_N', 10)
        self.pub_loadcell_comp = self.create_publisher(
            Float32, '/debug/loadcell_comp_N', 10)
        self.pub_loadcell_raw = self.create_publisher(
            Float32, '/debug/loadcell_raw_N', 10)

        # 圧力系
        self.pub_target_pH = self.create_publisher(
            Float32, '/debug/target_pressure_head_kPa', 10)
        self.pub_target_pR = self.create_publisher(
            Float32, '/debug/target_pressure_rod_kPa', 10)
        self.pub_current_pH = self.create_publisher(
            Float32, '/debug/current_pressure_head_kPa', 10)
        self.pub_current_pR = self.create_publisher(
            Float32, '/debug/current_pressure_rod_kPa', 10)

        # バルブ系
        self.pub_valve_uH = self.create_publisher(
            Float32, '/debug/valve_delta_head_V', 10)
        self.pub_valve_uR = self.create_publisher(
            Float32, '/debug/valve_delta_rod_V', 10)

        # ゲインスケジューリング
        self.pub_gain_ratio = self.create_publisher(
            Float32, '/debug/gain_ratio', 10)
        self.pub_sine_amp = self.create_publisher(
            Float32, '/debug/current_sine_amplitude_m', 10)

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
            f"Homing: head={self.get_parameter('homing_head_voltage').value}V, rod=0V. "
            f"Waiting for sensors..."
        )

    # ── PID 初期化 ───────────────────────────────────────────────
    def _init_pid(self):
        pos_limit  = float(self.get_parameter('pos_output_limit').value)
        pres_limit = float(self.get_parameter('pres_output_limit').value)

        self.pid_pos = PIDController(output_limit=pos_limit)
        self.pid_pH  = PIDController(output_limit=pres_limit)
        self.pid_pR  = PIDController(output_limit=pres_limit)
        self._update_gains()

    def _update_gains(self):
        final_kp = float(self.get_parameter('pos_kp').value)
        final_ki = float(self.get_parameter('pos_ki').value)
        final_kd = float(self.get_parameter('pos_kd').value)

        r = self._current_gain_ratio
        self.pid_pos.kp = final_kp * r
        self.pid_pos.ki = final_ki * r
        self.pid_pos.kd = final_kd * r
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

    def _update_gain_ramp(self, now):
        start_ratio = float(self.get_parameter('gain_ramp_start_ratio').value)
        duration    = float(self.get_parameter('gain_ramp_duration_s').value)

        if self.run_start_time is None:
            self._current_gain_ratio = start_ratio
            return

        elapsed = now - self.run_start_time
        if duration <= 0.0 or elapsed >= duration:
            self._current_gain_ratio = 1.0
        else:
            t = elapsed / duration
            self._current_gain_ratio = start_ratio + (1.0 - start_ratio) * t

    def _compose_target_force(self, pos_force_N, now, allow_loadcell):
        """
        F_target = F_pid - gain * F_loadcell

        ロードセル符号: PAMに引っ張られている時 → 負
        よって: -gain * (負) = +gain*|F| → 押し出し方向に加算 → 正しい
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
                    "Loadcell compensation enabled but data timed out. "
                    "Compensation disabled temporarily."
                )
                self._loadcell_timeout_warned = True
            return self._clamp(pos_force_N, -limit, limit)

        if self._loadcell_timeout_warned:
            self.get_logger().info("Loadcell signal recovered.")
            self._loadcell_timeout_warned = False

        loadcell_force = 0.0 if self.current_loadcell is None else float(self.current_loadcell)
        loadcell_gain  = float(self.get_parameter('loadcell_ff_gain').value)

        # F_target = F_pid - gain * F_loadcell
        comp_force = -loadcell_gain * loadcell_force
        self._last_loadcell_comp_N = comp_force

        return self._clamp(pos_force_N + comp_force, -limit, limit)

    # ── デバッグ一括パブリッシュ ──────────────────────────────────
    def _publish_debug(self, x_ref_rel, x_rel):
        self.pub_target_pos.publish(Float32(data=float(self.x_0 + x_ref_rel)))
        self.pub_current_rel_pos.publish(Float32(data=float(x_rel)))
        self.pub_position_error.publish(Float32(data=float(x_ref_rel - x_rel)))

        self.pub_target_force.publish(Float32(data=float(self._target_force_N)))
        self.pub_pid_force.publish(Float32(data=float(self._last_pos_force_N)))
        self.pub_loadcell_comp.publish(Float32(data=float(self._last_loadcell_comp_N)))
        self.pub_loadcell_raw.publish(Float32(
            data=float(0.0 if self.current_loadcell is None else self.current_loadcell)))

        self.pub_target_pH.publish(Float32(data=float(self._last_target_pH)))
        self.pub_target_pR.publish(Float32(data=float(self._last_target_pR)))
        self.pub_current_pH.publish(Float32(data=float(self.current_pH)))
        self.pub_current_pR.publish(Float32(data=float(self.current_pR)))

        self.pub_valve_uH.publish(Float32(data=float(self._last_uH)))
        self.pub_valve_uR.publish(Float32(data=float(self._last_uR)))

        self.pub_gain_ratio.publish(Float32(data=float(self._current_gain_ratio)))
        self.pub_sine_amp.publish(Float32(data=float(self.current_sine_amp)))

    # ────────────────────────────────────────────────────────────
    # 外側ループ
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

        self._update_gain_ramp(now)
        self._update_gains()

        if self.state == ControllerState.RUNNING:
            self._state_running(now, dt)
        else:
            self._target_force_N = 0.0
            self._send_all_neutral()

    # ────────────────────────────────────────────────────────────
    # 内側ループ
    # ────────────────────────────────────────────────────────────
    def _inner_loop(self):
        if self.state != ControllerState.RUNNING:
            return

        self._apply_force_to_valves(self._target_force_N, self._inner_dt)

    # ────────────────────────────────────────────────────────────
    # 各状態の処理
    # ────────────────────────────────────────────────────────────
    def _state_waiting_sensor(self, now):
        homing_v = float(self.get_parameter('homing_head_voltage').value)
        self._send_valve(homing_v, 0.0)

        if self.current_pos is not None:
            self.get_logger().info(
                f"Sensors connected. Homing: head={homing_v}V, rod=0V"
            )
            self.state = ControllerState.HOMING
            self.homing_start_time = now
            self.homing_last_pos = self.current_pos
            self.homing_settle_start = None

    def _state_homing(self, now):
        settle_thresh = float(self.get_parameter('homing_settle_threshold').value)
        settle_dur    = float(self.get_parameter('homing_settle_duration').value)
        startup_wait  = float(self.get_parameter('homing_startup_wait').value)
        homing_v      = float(self.get_parameter('homing_head_voltage').value)

        self._send_valve(homing_v, 0.0)

        if now - self.homing_start_time < startup_wait:
            self.homing_last_pos = self.current_pos
            return

        pos_change = abs(self.current_pos - self.homing_last_pos)

        if pos_change < settle_thresh:
            if self.homing_settle_start is None:
                self.homing_settle_start = now
            elif now - self.homing_settle_start > settle_dur:
                self.x_0 = self.current_pos
                self.get_logger().info(
                    f"Homing complete. x_0 = {self.x_0:.4f} m (sine minimum end). "
                    f"Starting RUNNING with gain ramp."
                )

                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self._target_force_N     = 0.0
                self._current_gain_ratio = float(
                    self.get_parameter('gain_ramp_start_ratio').value
                )
                self._outer_last_time = time.monotonic()
                self.run_start_time   = time.monotonic()
                self.current_sine_amp = 0.0

                self.state = ControllerState.RUNNING
                return
        else:
            self.homing_settle_start = None

        self.homing_last_pos = self.current_pos

    def _state_running(self, now, dt):
        """
        x_ref_rel = A * (1 - cos(ωt))

        t=0    →  0    (x_0, 最小端)
        t=T/4  → +A    (振動中心)
        t=T/2  → +2A   (最大端)
        t=3T/4 → +A    (振動中心)
        t=T    →  0    (x_0 に戻る)
        """
        target_amp = max(0.0, float(self.get_parameter('sine_amplitude_m').value))
        freq       = float(self.get_parameter('sine_freq_hz').value)
        amp_rate   = max(0.0, float(self.get_parameter('sine_amp_ramp_rate_m_s').value))

        elapsed = now - self.run_start_time

        delta_amp = amp_rate * dt
        if self.current_sine_amp < target_amp:
            self.current_sine_amp = min(self.current_sine_amp + delta_amp, target_amp)
        else:
            self.current_sine_amp = max(self.current_sine_amp - delta_amp, target_amp)

        A = self.current_sine_amp

        x_ref_rel = A * (1.0 - math.cos(2.0 * math.pi * freq * elapsed))

        x_rel = self._get_relative_pos()

        pos_force = self.pid_pos.update(x_ref_rel, x_rel, dt)
        self._target_force_N = self._compose_target_force(
            pos_force, now, allow_loadcell=True
        )

        self._publish_debug(x_ref_rel, x_rel)

    # ────────────────────────────────────────────────────────────
    # 推力 → 圧力目標 → 圧力 PI → バルブ電圧
    # ────────────────────────────────────────────────────────────
    def _apply_force_to_valves(self, target_force_N, dt):
        base_kpa = float(self.get_parameter('base_pressure_kpa').value)
        max_kpa  = float(self.get_parameter('supply_pressure_kpa').value)

        delta_pH =  target_force_N / self.AREA_HEAD / 1000.0
        delta_pR = -target_force_N / self.AREA_ROD  / 1000.0

        target_pH = self._clamp(base_kpa + delta_pH, 0.0, max_kpa)
        target_pR = self._clamp(base_kpa + delta_pR, 0.0, max_kpa)

        uH = self.pid_pH.update(target_pH, self.current_pH, dt)
        uR = self.pid_pR.update(target_pR, self.current_pR, dt)

        self._send_valve(self.VALVE_NEUTRAL + uH, self.VALVE_NEUTRAL + uR)

        # デバッグ用に保持
        self._last_target_pH = target_pH
        self._last_target_pR = target_pR
        self._last_uH = uH
        self._last_uR = uR


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
