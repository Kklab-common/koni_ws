import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import time


class PIController:
    def __init__(self, kp=0.0, ki=0.0, output_limit=4.9):
        self.kp = kp
        self.ki = ki
        self.output_limit = output_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, target, actual, dt):
        if dt <= 0.0:
            return 0.0
        error = target - actual
        self.integral += error * dt
        if self.ki > 1e-9:
            integral_limit = self.output_limit / self.ki
            self.integral = max(-integral_limit, min(integral_limit, self.integral))
        output = self.kp * error + self.ki * self.integral
        return max(-self.output_limit, min(self.output_limit, output))


class PamPressureController(Node):
    """
    PAM圧力制御ノード。

    入力:
      /sensors/pam_pressure : Float32 [kPa]
    出力:
      /actuators/pam_valve  : Float32MultiArray [ch, volt]

    PI制御により pam_pressure を target_pressure_kpa に保つ。
    バルブは 0–10V, 5V 中立。
    """

    VALVE_NEUTRAL = 5.0

    def __init__(self):
        super().__init__('pam_pressure_controller')

        self.declare_parameter('target_pressure_kpa', 100.0)
        self.declare_parameter('kp', 0.02)
        self.declare_parameter('ki', 0.005)
        self.declare_parameter('output_limit', 4.9)
        self.declare_parameter('valve_channel', 3)
        self.declare_parameter('control_rate_hz', 500.0)
        self.declare_parameter('pressure_topic', '/sensors/pam_pressure')
        self.declare_parameter('valve_topic', '/actuators/pam_valve')

        kp             = float(self.get_parameter('kp').value)
        ki             = float(self.get_parameter('ki').value)
        output_limit   = float(self.get_parameter('output_limit').value)
        self.channel   = int(self.get_parameter('valve_channel').value)
        rate_hz        = float(self.get_parameter('control_rate_hz').value)
        pressure_topic = self.get_parameter('pressure_topic').value
        valve_topic    = self.get_parameter('valve_topic').value

        self.pi = PIController(kp=kp, ki=ki, output_limit=output_limit)

        self.current_pressure = None
        self._last_time = None

        self.pub_valve        = self.create_publisher(Float32MultiArray, valve_topic, 10)
        self.pub_debug_target = self.create_publisher(Float32, '/debug/pam_target_pressure_kPa', 10)
        self.pub_debug_error  = self.create_publisher(Float32, '/debug/pam_pressure_error_kPa', 10)
        self.pub_debug_output = self.create_publisher(Float32, '/debug/pam_valve_output_V', 10)

        self.create_subscription(Float32, pressure_topic, self._cb_pressure, 10)
        self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f"PAM Pressure Controller started. "
            f"target={self.get_parameter('target_pressure_kpa').value:.1f} kPa, "
            f"kp={kp}, ki={ki}, ch={self.channel}"
        )

    def _cb_pressure(self, msg: Float32):
        self.current_pressure = float(msg.data)

    def _control_loop(self):
        now = time.monotonic()

        if self.current_pressure is None:
            return

        if self._last_time is None:
            self._last_time = now
            return

        dt = now - self._last_time
        self._last_time = now
        if dt <= 0.0:
            return

        target_kpa = float(self.get_parameter('target_pressure_kpa').value)
        u = self.pi.update(target_kpa, self.current_pressure, dt)

        valve_volt = max(0.0, min(10.0, self.VALVE_NEUTRAL + u))

        msg = Float32MultiArray()
        msg.data = [float(self.channel), valve_volt]
        self.pub_valve.publish(msg)

        self.pub_debug_target.publish(Float32(data=target_kpa))
        self.pub_debug_error.publish(Float32(data=target_kpa - self.current_pressure))
        self.pub_debug_output.publish(Float32(data=u))


def main(args=None):
    rclpy.init(args=args)
    node = PamPressureController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        msg = Float32MultiArray()
        msg.data = [float(node.channel), PamPressureController.VALVE_NEUTRAL]
        node.pub_valve.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
