import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ValveMixer(Node):
    """
    複数の制御ノードからのバルブ指令を統合し、
    8chのバルブ電圧として一定周期でパブリッシュする。

    入力トピックのメッセージ形式 (Float32MultiArray):
        data = [ch, volt, ch, volt, ...]
        例: [0, 6.5, 1, 3.2]  → ch0=6.5V, ch1=3.2V

    出力:
        /actuators/valve_voltage : Float32MultiArray (8ch)
    """

    NUM_CHANNELS  = 8
    VALVE_NEUTRAL = 5.0

    def __init__(self):
        super().__init__('valve_mixer')

        self.declare_parameter('output_rate_hz', 1000.0)
        self.declare_parameter('input_topics', [
            '/actuators/cylinder_valves',
            '/actuators/pam_valve',
        ])

        rate_hz      = float(self.get_parameter('output_rate_hz').value)
        input_topics = self.get_parameter('input_topics').value

        # 各チャンネルの最新電圧（中立で初期化）
        self._voltages = [self.VALVE_NEUTRAL] * self.NUM_CHANNELS

        # 出力パブリッシャ
        self.pub_valve = self.create_publisher(
            Float32MultiArray, '/actuators/valve_voltage', 10)

        # 入力サブスクライバ（トピックごとに生成）
        for topic in input_topics:
            self.create_subscription(
                Float32MultiArray, topic,
                self._make_callback(topic), 10)
            self.get_logger().info(f"Subscribed to: {topic}")

        # 一定周期で統合出力
        self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f"ValveMixer started. "
            f"output_rate={rate_hz:.0f}Hz, "
            f"inputs={input_topics}"
        )

    def _make_callback(self, topic):
        def callback(msg: Float32MultiArray):
            data = msg.data
            if len(data) % 2 != 0:
                self.get_logger().warn(
                    f"[{topic}] Odd number of elements ({len(data)}). Ignoring.")
                return
            for i in range(0, len(data), 2):
                ch   = int(data[i])
                volt = float(data[i + 1])
                if 0 <= ch < self.NUM_CHANNELS:
                    self._voltages[ch] = max(0.0, min(10.0, volt))
                else:
                    self.get_logger().warn(
                        f"[{topic}] Invalid channel {ch}. Ignoring.")
        return callback

    def _publish(self):
        msg = Float32MultiArray()
        msg.data = list(self._voltages)
        self.pub_valve.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ValveMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        # 全チャンネル中立に戻す
        msg = Float32MultiArray()
        msg.data = [ValveMixer.VALVE_NEUTRAL] * ValveMixer.NUM_CHANNELS
        node.pub_valve.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
