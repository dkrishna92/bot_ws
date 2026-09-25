"""Bosch BNO055 IMU driver node.

Interface decided: I2C, on the Raspberry Pi's hardware I2C1 bus (GPIO2/
SDA1, GPIO3/SCL1 -- header pins 3/5). This was the open interface-choice
item in CLAUDE.md's Hardware section; the mounting location and NDOF
magnetometer-near-motors caution noted there still apply and haven't been
re-verified against real hardware here.

Publishes sensor_msgs/Imu on "imu", matching bot_bringup/config/
ekf_params.yaml's imu0 topic. The chip's own NDOF fusion mode is used, so
orientation is the BNO055's absolute, magnetically-referenced quaternion,
not integrated here -- same trust assumption ekf_params.yaml already
documents (imu0_relative: false).

Uses smbus2 for I2C, same "import guarded, dry-run on non-Pi dev
machines" pattern as bot_motor's RPi.GPIO usage.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None  # allows import on dev machines without an I2C bus

# BNO055 register map (Bosch BNO055 datasheet section 4.2)
_CHIP_ID_ADDR = 0x00
_EXPECTED_CHIP_ID = 0xA0
_ACC_DATA_X_LSB_ADDR = 0x08
_GYR_DATA_X_LSB_ADDR = 0x14
_QUATERNION_DATA_W_LSB_ADDR = 0x20
_UNIT_SEL_ADDR = 0x3B
_OPR_MODE_ADDR = 0x3D
_PWR_MODE_ADDR = 0x3E

_CONFIG_MODE = 0x00
_NDOF_MODE = 0x0C
_NORMAL_POWER_MODE = 0x00
_UNIT_SEL_GYRO_RPS = 0x02  # bit1=1: gyro output in rad/s instead of deg/s

_ACC_LSB_PER_MS2 = 100.0
_GYR_LSB_PER_RPS = 900.0
_QUA_LSB_PER_UNIT = 1.0 / (1 << 14)

# Approximated from the same BNO055 datasheet noise-density figures as
# bot_gazebo/urdf/bot.urdf.xacro's simulated IMU noise -- see that file's
# comment for the density->stddev derivation. Orientation covariance has
# no equivalent datasheet noise-density spec (it's the chip's own fused
# NDOF output, not a raw channel), so this is a rough placeholder pending
# real-hardware calibration -- same "confirm before locking" status as
# other open items in CLAUDE.md.
_ORIENTATION_STDDEV_RAD = 0.02
_ANGULAR_VELOCITY_STDDEV_RAD_S = 0.0017
_LINEAR_ACCELERATION_STDDEV_M_S2 = 0.0104


def _diag_covariance(stddev: float) -> list[float]:
    var = stddev * stddev
    return [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]


class Bno055Node(Node):
    def __init__(self):
        super().__init__("bno055_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x28)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 50.0)

        self._address = self.get_parameter("i2c_address").value
        self._frame_id = self.get_parameter("frame_id").value

        self._orientation_covariance = _diag_covariance(_ORIENTATION_STDDEV_RAD)
        self._angular_velocity_covariance = _diag_covariance(_ANGULAR_VELOCITY_STDDEV_RAD_S)
        self._linear_acceleration_covariance = _diag_covariance(_LINEAR_ACCELERATION_STDDEV_M_S2)

        self._publisher = self.create_publisher(Imu, "imu", 10)

        if SMBus is not None:
            self._bus = SMBus(self.get_parameter("i2c_bus").value)
            self._init_sensor()
        else:
            self._bus = None
            self.get_logger().warn("smbus2 not available -- running in dry-run mode. "
                                    "Expected on a non-Pi dev machine.")

        rate_hz = self.get_parameter("rate_hz").value
        self.create_timer(1.0 / rate_hz, self._tick)

    def _init_sensor(self) -> None:
        chip_id = self._bus.read_byte_data(self._address, _CHIP_ID_ADDR)
        if chip_id != _EXPECTED_CHIP_ID:
            self.get_logger().error(
                f"BNO055 chip ID mismatch: expected 0x{_EXPECTED_CHIP_ID:02X}, "
                f"got 0x{chip_id:02X} -- check wiring/address")

        self._bus.write_byte_data(self._address, _OPR_MODE_ADDR, _CONFIG_MODE)
        time.sleep(0.02)
        self._bus.write_byte_data(self._address, _PWR_MODE_ADDR, _NORMAL_POWER_MODE)
        self._bus.write_byte_data(self._address, _UNIT_SEL_ADDR, _UNIT_SEL_GYRO_RPS)
        self._bus.write_byte_data(self._address, _OPR_MODE_ADDR, _NDOF_MODE)
        time.sleep(0.02)

    def _read_i16(self, addr: int) -> int:
        lsb, msb = self._bus.read_i2c_block_data(self._address, addr, 2)
        raw = (msb << 8) | lsb
        return raw - 0x10000 if raw & 0x8000 else raw

    def _tick(self) -> None:
        if self._bus is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        qw = self._read_i16(_QUATERNION_DATA_W_LSB_ADDR) * _QUA_LSB_PER_UNIT
        qx = self._read_i16(_QUATERNION_DATA_W_LSB_ADDR + 2) * _QUA_LSB_PER_UNIT
        qy = self._read_i16(_QUATERNION_DATA_W_LSB_ADDR + 4) * _QUA_LSB_PER_UNIT
        qz = self._read_i16(_QUATERNION_DATA_W_LSB_ADDR + 6) * _QUA_LSB_PER_UNIT
        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation_covariance = self._orientation_covariance

        msg.angular_velocity.x = self._read_i16(_GYR_DATA_X_LSB_ADDR) / _GYR_LSB_PER_RPS
        msg.angular_velocity.y = self._read_i16(_GYR_DATA_X_LSB_ADDR + 2) / _GYR_LSB_PER_RPS
        msg.angular_velocity.z = self._read_i16(_GYR_DATA_X_LSB_ADDR + 4) / _GYR_LSB_PER_RPS
        msg.angular_velocity_covariance = self._angular_velocity_covariance

        msg.linear_acceleration.x = self._read_i16(_ACC_DATA_X_LSB_ADDR) / _ACC_LSB_PER_MS2
        msg.linear_acceleration.y = self._read_i16(_ACC_DATA_X_LSB_ADDR + 2) / _ACC_LSB_PER_MS2
        msg.linear_acceleration.z = self._read_i16(_ACC_DATA_X_LSB_ADDR + 4) / _ACC_LSB_PER_MS2
        msg.linear_acceleration_covariance = self._linear_acceleration_covariance

        self._publisher.publish(msg)

    def destroy_node(self) -> None:
        if self._bus is not None:
            self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Bno055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
