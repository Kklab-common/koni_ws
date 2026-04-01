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
        両室大気開放 (0V, 0V) → PAM引張力で伸び切った位置 = x_0
        x_0 は正弦波の最大端。

    RUNNING（ホーミング直後に開始）:
        x_ref_rel = -A * (1 - cos(ωt))

        振幅 A は 0 から徐々にランプで増加するため、
        x_0 付近から滑らかに振動が始まる。

        t=0    → x_ref_rel =  0    (x_0: PAM自然位置, 最大端)
        t=T/4  → x_ref_rel = -A    (振動中心)
        t=T/2  → x_ref_rel = -2A   (最小端)
        t=3T/4 → x_ref_rel = -A    (振動中心)
        t=T    → x_ref_rel =  0    (x_0 に戻る)
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

        # 軌道
        self.declare_parameter('sine_amplitude_m', 0.020)
        self.declare_parameter('sine_freq_hz', 0.5)

        # 振幅ランプ
        self.declare_parameter('sine_amp_ramp_rate_m_s', 0.010)

        # 圧力
        self.declare_parameter('base_pressure_kpa', 250.0)
        self.declare_parameter('supply_pressure_kpa', 600.0)

        # 位置ループ PID（外側）
        self.declare_parameter('pos_kp', 2000.0)
        self.declare_parameter('pos_ki', 0.0)
        self.declare_parameter('pos_kd', 0.0)
        self.declare_parameter('pos_td', 0.005)
        self.declare_parameter('pos_output_limit', 1000.0)

        # 圧力ループ PI（内側）
        self.declare_parameter('pres_kp', 0.02)
        self.declare_parameter('pres_ki', 0.005)
        self.declare_parameter('pres_kd', 0.0)
        self.declare_parameter('pres_td', 0.01)
        self.declare_parameter('pres_output_limit', 4.9)

        # ホーミング
        self.declare_parameter('homing_settle_threshold', 0.0002)
        self.declare_parameter('homing_settle_duration', 1.0)
        self.declare_parameter('homing_startup_wait', 0.5)

        # ロードセル補償
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

        # ホーミング用
        self.homing_start_time   = None
        self.homing_last_pos     = None
        self.homing_settle_start = None

        # RUNNING 用
        self.current_sine_amp = 0.0
        self.run_start_time   = None

        self._outer_last_time = None
        self._target_force_N  = 0.0

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
            f"HOMING → RUNNING (no PRESSURIZE). "
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

    def _send_all_exhaust(self):
        self._send_valve(0.0, 5.2)

    def _get_relative_pos(self):
        return self.current_pos - self.x_0

    def _compose_target_force(self, pos_force_N, now, allow_loadcell):
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

        comp_force = loadcell_gain * loadcell_force
        self._last_loadcell_comp_N = comp_force

        return self._clamp(pos_force_N + comp_force, -limit, limit)

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
        self._send_all_exhaust()

        if self.current_pos is not None:
            self.get_logger().info(
                "Sensors connected. Exhausting both chambers → "
                "finding PAM natural position..."
            )
            self.state = ControllerState.HOMING
            self.homing_start_time = now
            self.homing_last_pos = self.current_pos
            self.homing_settle_start = None

    def _state_homing(self, now):
        settle_thresh = float(self.get_parameter('homing_settle_threshold').value)
        settle_dur    = float(self.get_parameter('homing_settle_duration').value)
        startup_wait  = float(self.get_parameter('homing_startup_wait').value)

        self._send_all_exhaust()

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
                    f"Homing complete. x_0 = {self.x_0:.4f} m (sine max end). "
                    f"Starting RUNNING immediately."
                )

                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self._target_force_N  = 0.0
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
        片側正弦波追従。

        x_ref_rel = -A * (1 - cos(ωt))

        t=0    →  0   (x_0, 最大端 = スタート地点)
        t=T/4  → -A   (振動中心)
        t=T/2  → -2A  (最小端, 最も縮んだ位置)
        t=3T/4 → -A   (振動中心)
        t=T    →  0   (x_0 に戻る)

        A は 0 から振幅ランプで徐々に増加するため、
        スタート直後は x_0 付近で微小振動し、滑らかに振幅が広がる。
        """
        target_amp = max(0.0, float(self.get_parameter('sine_amplitude_m').value))
        freq       = float(self.get_parameter('sine_freq_hz').value)
        amp_rate   = max(0.0, float(self.get_parameter('sine_amp_ramp_rate_m_s').value))

        elapsed = now - self.run_start_time

        # 振幅ランプ
        delta_amp = amp_rate * dt
        if self.current_sine_amp < target_amp:
            self.current_sine_amp = min(self.current_sine_amp + delta_amp, target_amp)
        else:
            self.current_sine_amp = max(self.current_sine_amp - delta_amp, target_amp)

        A = self.current_sine_amp

        # x_0 が最大端、そこから伸びる向のみ
        x_ref_rel = A * (1.0 - math.cos(2.0 * math.pi * freq * elapsed))

        x_rel = self._get_relative_pos()

        pos_force = self.pid_pos.update(x_ref_rel, x_rel, dt)
        self._target_force_N = self._compose_target_force(
            pos_force, now, allow_loadcell=True
        )

        self.pub_target_pos.publish(Float32(data=float(self.x_0 + x_ref_rel)))
        self.pub_current_rel_pos.publish(Float32(data=float(x_rel)))

    # ────────────────────────────────────────────────────────────
    # 推力 → 圧力目標 → 圧力 PI → バルブ電圧
    # ────────────────────────────────────────────────────────────
    def _apply_force_to_valves(self, target_force_N, dt):
        base_kpa = float(self.get_parameter('base_pressure_kpa').value)
        max_kpa  = float(self.get_parameter('supply_pressure_kpa').value)

        target_pH = base_kpa
        target_pR = base_kpa

        if target_force_N > 0.0:
            target_pH += target_force_N / self.AREA_HEAD / 1000.0
        elif target_force_N < 0.0:
            target_pR += (-target_force_N) / self.AREA_ROD / 1000.0

        target_pH = self._clamp(target_pH, 0.0, max_kpa)
        target_pR = self._clamp(target_pR, 0.0, max_kpa)

        uH = self.pid_pH.update(target_pH, self.current_pH, dt)
        uR = self.pid_pR.update(target_pR, self.current_pR, dt)

        self._send_valve(self.VALVE_NEUTRAL + uH, self.VALVE_NEUTRAL + uR)

        msg = Float32MultiArray()
        msg.data = [
            float(target_pH),
            float(self.current_pH),
            float(target_pR),
            float(self.current_pR),
            float(uH),
            float(uR),
            float(target_force_N),
            float(0.0 if self.current_loadcell is None else self.current_loadcell),
            float(self._last_pos_force_N),
            float(self._last_loadcell_comp_N),
            float(self.current_sine_amp),
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
