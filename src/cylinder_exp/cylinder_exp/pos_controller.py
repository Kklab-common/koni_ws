import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import math
import time
from enum import Enum, auto


class ControllerState(Enum):
    WAITING_SENSOR = auto()
    HOMING = auto()
    PRESSURIZE = auto()
    RUNNING = auto()
    STOPPED = auto()


class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, td=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.td = td

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.output_limit = 1000.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def update(self, target, actual, dt):
        if dt <= 0.0:
            return 0.0

        error = target - actual

        # 積分 + アンチワインドアップ
        self.integral += error * dt
        if self.ki > 1e-9:
            integral_limit = self.output_limit / self.ki
            self.integral = max(-integral_limit, min(integral_limit, self.integral))

        # 不完全微分
        raw_derivative = (error - self.prev_error) / dt
        alpha = self.td / (self.td + dt)
        filtered_derivative = alpha * self.prev_derivative + (1.0 - alpha) * raw_derivative

        output = self.kp * error + self.ki * self.integral + self.kd * filtered_derivative

        self.prev_error = error
        self.prev_derivative = filtered_derivative

        return max(-self.output_limit, min(self.output_limit, output))


class CylinderPositionController(Node):
    def __init__(self):
        super().__init__('cylinder_position_controller')

        # シリンダロッドの寸法
        D_cyl = 0.020
        d_rod = 0.010
        self.AREA_HEAD = math.pi / 4.0 * D_cyl**2
        self.AREA_ROD = math.pi / 4.0 * (D_cyl**2 - d_rod**2)
        self.declare_parameter('ch_head', 0)
        self.declare_parameter('ch_rod', 1)
        self.CH_HEAD = self.get_parameter('ch_head').value
        self.CH_ROD = self.get_parameter('ch_rod').value

        self.VALVE_NEUTRAL = 5.0

        # 軌道
        self.declare_parameter('sine_amplitude_m', 0.020)
        self.declare_parameter('sine_freq_hz', 0.5)
        self.declare_parameter('center_position_m', 0.060)

        # 圧力
        self.declare_parameter('base_pressure_kpa', 250.0)
        self.declare_parameter('supply_pressure_kpa', 600.0)

        # 位置ループPID
        self.declare_parameter('pos_kp', 2000.0)
        self.declare_parameter('pos_ki', 500.0)
        self.declare_parameter('pos_kd', 100.0)
        self.declare_parameter('pos_td', 0.005)

        # 圧力ループPI
        self.declare_parameter('pres_kp', 0.02)
        self.declare_parameter('pres_ki', 0.005)
        self.declare_parameter('pres_kd', 0.0)
        self.declare_parameter('pres_td', 0.01)

        # ホーミング
        self.declare_parameter('homing_settle_threshold', 0.0002)
        self.declare_parameter('homing_settle_duration', 1.0)
        self.declare_parameter('homing_startup_wait', 0.5)

        # PRESSURIZE → RUNNING 遷移
        self.declare_parameter('pressurize_pos_threshold', 0.002)
        self.declare_parameter('pressurize_settle_duration', 0.5)

        # 状態変数
        self.state = ControllerState.WAITING_SENSOR
        self.x_0 = 0.0
        self.current_pos = None
        self.current_pH = 0.0
        self.current_pR = 0.0

        # ホーミング用
        self.homing_start_time = None
        self.homing_last_pos = None
        self.homing_settle_start = None

        # PRESSURIZE
        self.pressurize_settle_start = None

        self.run_start_time = None
        self.last_time = None

        # PID
        self.pid_pos = PIDController()
        self.pid_pH = PIDController()
        self.pid_pR = PIDController()

        # パブリッシャ
        self.pub_valve = self.create_publisher(
            Float32MultiArray, '/actuators/valve_voltage', 10)
        self.pub_debug = self.create_publisher(
            Float32MultiArray, '/debug/cylinder_controller', 10)
        
        # 目標軌道と現在位置の専用トピックを追加
        self.pub_target_pos = self.create_publisher(
            Float32, '/control/target_position', 10)
        self.pub_current_rel_pos = self.create_publisher(
            Float32, '/control/current_rel_position', 10)

        # サブスクライバ (トピック名はPython版のまま維持)
        self.create_subscription(
            Float32, '/sensors/cylinder_position', self._cb_pos, 10)
        self.create_subscription(
            Float32, '/sensors/head_pressure', self._cb_ph, 10)
        self.create_subscription(
            Float32, '/sensors/rod_pressure', self._cb_pr, 10)

        self.timer = self.create_timer(0.01, self._control_loop)
        self.get_logger().info("Controller initialized. Waiting for sensors...")

    # センサコールバック
    def _cb_pos(self, msg):
        self.current_pos = msg.data

    def _cb_ph(self, msg):
        self.current_pH = msg.data

    def _cb_pr(self, msg):
        self.current_pR = msg.data

    # ユーティリティ
    def _send_valve(self, volt_h, volt_r):
        volt_h = max(0.0, min(10.0, volt_h))
        volt_r = max(0.0, min(10.0, volt_r))
        msg = Float32MultiArray()
        msg.data = [self.VALVE_NEUTRAL] * 8
        msg.data[self.CH_HEAD] = volt_h
        msg.data[self.CH_ROD] = volt_r
        self.pub_valve.publish(msg)

    def _send_all_neutral(self):
        self._send_valve(self.VALVE_NEUTRAL, self.VALVE_NEUTRAL)

    def _update_gains(self):
        self.pid_pos.kp = self.get_parameter('pos_kp').value
        self.pid_pos.ki = self.get_parameter('pos_ki').value
        self.pid_pos.kd = self.get_parameter('pos_kd').value
        self.pid_pos.td = self.get_parameter('pos_td').value

        for pid in [self.pid_pH, self.pid_pR]:
            pid.kp = self.get_parameter('pres_kp').value
            pid.ki = self.get_parameter('pres_ki').value
            pid.kd = self.get_parameter('pres_kd').value
            pid.td = self.get_parameter('pres_td').value

    def _get_relative_pos(self):
        return self.current_pos - self.x_0

    # 状態遷移メインループ
    def _control_loop(self):
        if self.state == ControllerState.WAITING_SENSOR:
            self._state_waiting_sensor()
        elif self.state == ControllerState.HOMING:
            self._state_homing()
        elif self.state == ControllerState.PRESSURIZE:
            self._state_pressurize()
        elif self.state == ControllerState.RUNNING:
            self._state_running()
        else:
            self._send_all_neutral()

    # 各状態の処理
    def _state_waiting_sensor(self):
        self._send_all_neutral()
        if self.current_pos is not None:
            self.get_logger().info("Sensors connected. Starting homing...")
            self.state = ControllerState.HOMING
            self.homing_start_time = time.time()
            self.homing_last_pos = self.current_pos
            self.homing_settle_start = None

    def _state_homing(self):
        settle_thresh = self.get_parameter('homing_settle_threshold').value
        settle_dur = self.get_parameter('homing_settle_duration').value
        startup_wait = self.get_parameter('homing_startup_wait').value

        self._send_valve(0.0, 10.0)

        now = time.time()

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
                    f"Homing complete. Origin = {self.x_0:.4f} m")
                self.state = ControllerState.PRESSURIZE
                self.pressurize_settle_start = None
                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self.last_time = time.time()
                return
        else:
            self.homing_settle_start = None

        self.homing_last_pos = self.current_pos

    def _state_pressurize(self):
        """
        位置制御で中心位置まで移動。
        目標位置付近に安定したらRUNNINGに遷移。
        """
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            return
        self.last_time = now
        self._update_gains()

        center = self.get_parameter('center_position_m').value
        pos_thresh = self.get_parameter('pressurize_pos_threshold').value
        settle_dur = self.get_parameter('pressurize_settle_duration').value

        x_rel = self._get_relative_pos()
        target_force = self.pid_pos.update(center, x_rel, dt)
        self._apply_force_to_valves(target_force, dt)

        # 専用トピックに出力
        msg_tgt = Float32()
        msg_tgt.data = float(center)
        self.pub_target_pos.publish(msg_tgt)

        msg_cur = Float32()
        msg_cur.data = float(x_rel)
        self.pub_current_rel_pos.publish(msg_cur)

        # 安定判定
        if abs(x_rel - center) < pos_thresh:
            if self.pressurize_settle_start is None:
                self.pressurize_settle_start = now
            elif now - self.pressurize_settle_start > settle_dur:
                self.get_logger().info(
                    f"Center reached ({x_rel * 1000:.1f} mm). Starting sine wave.")
                self.state = ControllerState.RUNNING
                self.run_start_time = time.time()
                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self.last_time = time.time()
                return
        else:
            self.pressurize_settle_start = None

    def _state_running(self):
        """
        正弦波追従制御。
        """
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            return
        self.last_time = now
        self._update_gains()

        amp = self.get_parameter('sine_amplitude_m').value
        freq = self.get_parameter('sine_freq_hz').value
        center = self.get_parameter('center_position_m').value

        elapsed = now - self.run_start_time
        omega = 2.0 * math.pi * freq

        # 目標軌道 (先輩の compute_sine_target と同じ)
        x_ref = center + amp * math.sin(omega * elapsed)

        # 位置PID → 目標推力
        x_rel = self._get_relative_pos()
        target_force = self.pid_pos.update(x_ref, x_rel, dt)

        # 推力 → 圧力 → バルブ
        self._apply_force_to_valves(target_force, dt)

        # 専用トピックに出力 (PlotJuggler用)
        msg_tgt = Float32()
        msg_tgt.data = float(x_ref)
        self.pub_target_pos.publish(msg_tgt)

        msg_cur = Float32()
        msg_cur.data = float(x_rel)
        self.pub_current_rel_pos.publish(msg_cur)


    # 推力→圧力→バルブ変換 (先輩のロジックを完全移植！)
    def _apply_force_to_valves(self, target_force_N, dt):
        """
        ★先輩の convert_force_to_target_pressures ロジック
        基本は両室ともベース圧力を維持し、
        動かしたい方向の部屋「だけ」圧力をプラスする方式。
        """
        base_pressure_kpa = self.get_parameter('base_pressure_kpa').value
        max_pressure_kpa = self.get_parameter('supply_pressure_kpa').value

        # 1. まずは両室ともベース圧力にする
        target_pH = base_pressure_kpa
        target_pR = base_pressure_kpa

        # 2. 推力の向きに応じて、片方だけ圧力を足す
        if target_force_N > 0.0:
            # 伸ばす方向: ヘッド側に圧力を足す (F = P * A -> P = F / A / 1000)
            target_pH += target_force_N / self.AREA_HEAD / 1000.0
        elif target_force_N < 0.0:
            # 縮む方向: ロッド側に圧力を足す (負の力なので - を付けて正にする)
            target_pR += (-target_force_N) / self.AREA_ROD / 1000.0

        # 3. 安全のためのクランプ
        target_pH = max(0.0, min(max_pressure_kpa, target_pH))
        target_pR = max(0.0, min(max_pressure_kpa, target_pR))

        # 4. 圧力PI → バルブ電圧
        uH = self.pid_pH.update(target_pH, self.current_pH, dt)
        uR = self.pid_pR.update(target_pR, self.current_pR, dt)

        self._send_valve(self.VALVE_NEUTRAL + uH, self.VALVE_NEUTRAL + uR)

        # デバッグ出力
        msg_debug = Float32MultiArray()
        msg_debug.data = [
            float(target_pH),       # [0] 目標ヘッド圧 (kPa)
            float(self.current_pH), # [1] 現在ヘッド圧 (kPa)
            float(target_pR),       # [2] 目標ロッド圧 (kPa)
            float(self.current_pR), # [3] 現在ロッド圧 (kPa)
            float(uH),              # [4] ヘッド側バルブ加算電圧 (V)
            float(uR)               # [5] ロッド側バルブ加算電圧 (V)
        ]
        self.pub_debug.publish(msg_debug)


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