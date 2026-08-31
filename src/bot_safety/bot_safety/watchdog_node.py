"""Software watchdog -- defense in depth, NOT the primary safety mechanism.

Same role as the earlier bare-Python prototype: the MCU is the actual <1s
real-time e-stop, isolated from this ROS 2 stack. This node just (a) mirrors
MCU e-stop status onto the ROS 2 graph via /system_fault so other nodes can
react, and (b) catches stack-level failures (a node dying) that the MCU
wouldn't know about on its own, by watching topic activity as a liveness
proxy instead of a dedicated heartbeat topic (simpler in ROS 2 since we get
topic statistics for free -- no need to hand-roll a heartbeat message type).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    import serial
except ImportError:
    serial = None


class WatchdogNode(Node):
    def __init__(self):
        super().__init__("watchdog_node")

        self.declare_parameter("fault_gpio_pin", 26)
        self.declare_parameter("mcu_serial_port", "/dev/ttyACM1")
        self.declare_parameter("mcu_baudrate", 115200)
        self.declare_parameter("scan_timeout_s", 1.0)

        self._fault_pin = self.get_parameter("fault_gpio_pin").value
        self._scan_timeout_s = self.get_parameter("scan_timeout_s").value

        self._pub = self.create_publisher(Bool, "system_fault", 10)
        self._last_scan_time = None
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)

        if GPIO is not None:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._fault_pin, GPIO.OUT)
            GPIO.output(self._fault_pin, False)

        self._mcu_serial = None
        self._mcu_estop_active = False
        if serial is not None:
            try:
                self._mcu_serial = serial.Serial(
                    self.get_parameter("mcu_serial_port").value,
                    self.get_parameter("mcu_baudrate").value,
                    timeout=0.05,
                )
            except serial.SerialException:
                self.get_logger().warn("could not open MCU status serial port -- "
                                        "continuing without MCU status mirroring")

        self.create_timer(0.05, self._tick)  # 20 Hz

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_time = self.get_clock().now()

    def _check_mcu_status(self) -> bool:
        if self._mcu_serial is None:
            return False
        try:
            line = self._mcu_serial.readline().decode(errors="ignore").strip()
            if line == "ESTOP":
                self._mcu_estop_active = True
            elif line == "OK":
                self._mcu_estop_active = False
        except serial.SerialException:
            self.get_logger().error("lost MCU serial connection")
        return self._mcu_estop_active

    def _tick(self) -> None:
        stale_lidar = False
        if self._last_scan_time is not None:
            age = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
            stale_lidar = age > self._scan_timeout_s

        mcu_estop = self._check_mcu_status()
        fault = stale_lidar or mcu_estop

        if GPIO is not None:
            GPIO.output(self._fault_pin, fault)

        if fault and stale_lidar:
            self.get_logger().error("lidar scan stale -- publishing fault")

        self._pub.publish(Bool(data=fault))

    def destroy_node(self) -> None:
        if GPIO is not None:
            GPIO.output(self._fault_pin, True)  # fail safe: assert fault on exit
            GPIO.cleanup()
        if self._mcu_serial is not None:
            self._mcu_serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
