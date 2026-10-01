"""Bosch BNO055 IMU driver node.

Interface decided: I2C, on the Raspberry Pi's hardware I2C1 bus (GPIO2/
SDA1, GPIO3/SCL1 -- header pins 3/5). This was the open interface-choice
item in CLAUDE.md's Hardware section; the mounting location and NDOF
magnetometer-near-motors caution noted there still apply and haven't been
re-verified against real hardware here.

Publishes sensor_msgs/Imu on "imu", matching bot_bringup/config/
ekf_params.yaml's imu0 topic. Orientation is the chip's own fused
quaternion, not integrated here. Fusion mode (the fusion_mode param):
  - "imuplus" (default): gyro + accel only. Heading is relative to
    power-on and drifts slowly, but the motors can't disturb it.
  - "ndof": adds the magnetometer for an absolute heading. Switched away
    from 2026-09-29: on the robot the chip reported CALIB_STAT sys=0 mag=0
    (uncalibrated) with the motors ~9 cm away, and its heading re-snapped
    as the magnetometer calibration changed -- mapping runs came out as
    two copies of the room rotated ~25-30 deg apart.

Uses smbus2 for I2C, same "import guarded, dry-run on non-Pi dev
machines" pattern as bot_motor's RPi.GPIO usage.
"""
from __future__ import annotations

import math
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
_IMUPLUS_MODE = 0x08
_FUSION_MODES = {"imuplus": _IMUPLUS_MODE, "ndof": _NDOF_MODE}
_NORMAL_POWER_MODE = 0x00
_UNIT_SEL_GYRO_RPS = 0x02  # bit1=1: gyro output in rad/s instead of deg/s

_ACC_LSB_PER_MS2 = 100.0
_GYR_LSB_PER_RPS = 900.0
_QUA_LSB_PER_UNIT = 1.0 / (1 << 14)
_REINIT_AFTER_FAILURES = 100  # consecutive failed reads (~2 s at 50 Hz)
# Right after a mode switch the chip reports a placeholder quaternion
# (identity / all zeros) for ~1 s until fusion runs, then jumps to its real
# heading. The EKF (imu0_relative) zeroes on the FIRST orientation it gets,
# so publishing the placeholder made every launch start with a phantom turn
# (+132 deg on 2026-09-30, with the robot standing still). Hold samples back
# until the quaternion is real, for at most this long.
_FUSION_STARTUP_GATE_S = 2.0
# A single bad read can carry a wild orientation (+105 deg and back in one
# sample, 2026-09-30). Skip samples whose heading moved more than this
# beyond what the gyro measured; accept a jump that persists for more than
# _MAX_SKIPPED_JUMPS samples (e.g. the chip really reset).
_MAX_HEADING_JUMP_RAD = 0.26  # ~15 deg
_MAX_SKIPPED_JUMPS = 5

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
        self.declare_parameter("fusion_mode", "imuplus")  # see module docstring

        self._address = self.get_parameter("i2c_address").value
        self._frame_id = self.get_parameter("frame_id").value
        mode_name = self.get_parameter("fusion_mode").value
        if mode_name not in _FUSION_MODES:
            self.get_logger().error(f"unknown fusion_mode '{mode_name}' -- using imuplus")
            mode_name = "imuplus"
        self._fusion_mode = _FUSION_MODES[mode_name]

        self._orientation_covariance = _diag_covariance(_ORIENTATION_STDDEV_RAD)
        self._angular_velocity_covariance = _diag_covariance(_ANGULAR_VELOCITY_STDDEV_RAD_S)
        self._linear_acceleration_covariance = _diag_covariance(_LINEAR_ACCELERATION_STDDEV_M_S2)

        self._publisher = self.create_publisher(Imu, "imu", 10)
        self._read_failures = 0
        # Startup gate / glitch filter state (see _FUSION_STARTUP_GATE_S)
        self._init_time = time.monotonic()
        self._fusion_ready = False
        self._last_yaw: float | None = None
        self._last_yaw_time = 0.0
        self._skipped_jumps = 0

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
        self._bus.write_byte_data(self._address, _OPR_MODE_ADDR, self._fusion_mode)
        time.sleep(0.02)
        # (Re)arm the startup gate: fusion restarts with a placeholder.
        self._init_time = time.monotonic()
        self._fusion_ready = False
        self._last_yaw = None

    @staticmethod
    def _i16_at(block: list[int], offset: int) -> int:
        raw = (block[offset + 1] << 8) | block[offset]
        return raw - 0x10000 if raw & 0x8000 else raw

    def _tick(self) -> None:
        if self._bus is None:
            return

        # Accel (0x08), gyro (0x14) and quaternion (0x20-0x27) registers are
        # contiguous and span exactly 32 bytes -- one block read instead of
        # ten 2-byte reads (10x fewer I2C transactions per tick).
        try:
            block = self._bus.read_i2c_block_data(self._address, _ACC_DATA_X_LSB_ADDR, 32)
        except OSError as exc:
            # The BNO055 occasionally misses a read on the Pi's I2C (seen
            # 2026-09-29: TimeoutError errno 110). That used to kill the node,
            # leaving the EKF -- which takes heading only from here -- with
            # no heading for the rest of the run. Skip the sample instead.
            self._read_failures += 1
            if self._read_failures == 1 or self._read_failures % 50 == 0:
                self.get_logger().warn(f"IMU read failed ({exc}) -- {self._read_failures} in a row")
            if self._read_failures % _REINIT_AFTER_FAILURES == 0:
                # ~2 s of failures: the chip probably reset (it would then
                # be back in CONFIG mode), so set it up again. Its heading
                # restarts from zero, so the EKF heading may jump once.
                self.get_logger().error("IMU unresponsive -- re-initialising the BNO055")
                try:
                    self._init_sensor()
                except OSError as init_exc:
                    self.get_logger().error(f"re-init failed: {init_exc}")
            return
        if self._read_failures:
            self.get_logger().info(f"IMU reads recovered after {self._read_failures} failure(s)")
            self._read_failures = 0
        acc_off = 0
        gyr_off = _GYR_DATA_X_LSB_ADDR - _ACC_DATA_X_LSB_ADDR
        qua_off = _QUATERNION_DATA_W_LSB_ADDR - _ACC_DATA_X_LSB_ADDR

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.orientation.w = self._i16_at(block, qua_off) * _QUA_LSB_PER_UNIT
        msg.orientation.x = self._i16_at(block, qua_off + 2) * _QUA_LSB_PER_UNIT
        msg.orientation.y = self._i16_at(block, qua_off + 4) * _QUA_LSB_PER_UNIT
        msg.orientation.z = self._i16_at(block, qua_off + 6) * _QUA_LSB_PER_UNIT
        msg.orientation_covariance = self._orientation_covariance
        gyro_z = self._i16_at(block, gyr_off + 4) / _GYR_LSB_PER_RPS
        if not self._orientation_usable(msg.orientation, gyro_z):
            return

        msg.angular_velocity.x = self._i16_at(block, gyr_off) / _GYR_LSB_PER_RPS
        msg.angular_velocity.y = self._i16_at(block, gyr_off + 2) / _GYR_LSB_PER_RPS
        msg.angular_velocity.z = self._i16_at(block, gyr_off + 4) / _GYR_LSB_PER_RPS
        msg.angular_velocity_covariance = self._angular_velocity_covariance

        msg.linear_acceleration.x = self._i16_at(block, acc_off) / _ACC_LSB_PER_MS2
        msg.linear_acceleration.y = self._i16_at(block, acc_off + 2) / _ACC_LSB_PER_MS2
        msg.linear_acceleration.z = self._i16_at(block, acc_off + 4) / _ACC_LSB_PER_MS2
        msg.linear_acceleration_covariance = self._linear_acceleration_covariance

        self._publisher.publish(msg)

    def _orientation_usable(self, q, gyro_z: float) -> bool:
        """False for samples to hold back: the placeholder quaternion before
        fusion runs, and one-off heading jumps the gyro doesn't back up."""
        now = time.monotonic()
        if not self._fusion_ready:
            norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
            placeholder = (q.w, q.x, q.y, q.z) == (1.0, 0.0, 0.0, 0.0) or abs(norm - 1.0) > 0.1
            if placeholder and now - self._init_time < _FUSION_STARTUP_GATE_S:
                return False
            self._fusion_ready = True
            self.get_logger().info("BNO055 fusion running -- publishing /imu")
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._last_yaw is not None:
            expected = gyro_z * (now - self._last_yaw_time)
            jump = math.atan2(math.sin(yaw - self._last_yaw - expected), math.cos(yaw - self._last_yaw - expected))
            if abs(jump) > _MAX_HEADING_JUMP_RAD:
                if self._skipped_jumps < _MAX_SKIPPED_JUMPS:
                    self._skipped_jumps += 1
                    self.get_logger().warn(f"IMU heading jumped {math.degrees(jump):+.0f} deg beyond the gyro "
                                           "-- skipping the sample (bad read?)")
                    return False
                self.get_logger().warn(f"IMU heading jump of {math.degrees(jump):+.0f} deg persisted -- accepting it")
        self._skipped_jumps = 0
        self._last_yaw, self._last_yaw_time = yaw, now
        return True

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
