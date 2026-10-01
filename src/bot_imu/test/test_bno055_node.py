"""Behavior tests for bno055_node's Imu message construction.

Runs in dry-run mode (smbus2 absent) -- a fake bus answers _tick's one
32-byte block read (accel 0x08 .. quaternion 0x27) so its message-building
logic is exercised without a real I2C bus.
"""
import pytest
import rclpy

from bot_imu.bno055_node import Bno055Node, _QUA_LSB_PER_UNIT


@pytest.fixture
def node(monkeypatch):
    # Force dry-run even where smbus2 IS installed (the Pi): otherwise the
    # node opens its default I2C bus for real -- on the race Pi that's
    # I2C1, which times out -- and every test errors.
    monkeypatch.setattr("bot_imu.bno055_node.SMBus", None)
    rclpy.init()
    n = Bno055Node()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


class _FakeBus:
    """Serves read_i2c_block_data from a {register: signed 16-bit value} map."""

    def __init__(self, raw_by_addr):
        self._regs = {}
        for addr, value in raw_by_addr.items():
            value &= 0xFFFF
            self._regs[addr], self._regs[addr + 1] = value & 0xFF, value >> 8

    def read_i2c_block_data(self, address, start, length):
        return [self._regs.get(start + i, 0) for i in range(length)]

    def close(self):
        pass


def _capture_publisher(node):
    node._fusion_ready = True  # past the startup gate (tested separately below)
    captured = []
    node._publisher.publish = lambda msg: captured.append(msg)
    return captured


def test_tick_is_noop_without_a_bus(node):
    captured = _capture_publisher(node)
    node._tick()
    assert captured == []


def test_quaternion_and_rates_are_scaled_and_published(node):
    captured = _capture_publisher(node)
    raw_by_addr = {
        0x20: int(1.0 / _QUA_LSB_PER_UNIT),  # w
        0x22: 0,  # x
        0x24: 0,  # y
        0x26: 0,  # z
        0x14: 900,  # gyro x -> 1.0 rad/s (900 LSB/rps)
        0x16: 0,
        0x18: 0,
        0x08: 100,  # accel x -> 1.0 m/s^2 (100 LSB per m/s^2)
        0x0A: 0,
        0x0C: 0,
    }
    node._bus = _FakeBus(raw_by_addr)

    node._tick()

    assert len(captured) == 1
    msg = captured[0]
    assert msg.header.frame_id == "imu_link"
    assert msg.orientation.w == pytest.approx(1.0)
    assert msg.angular_velocity.x == pytest.approx(1.0)
    assert msg.linear_acceleration.x == pytest.approx(1.0)
    assert msg.orientation_covariance[0] == pytest.approx(0.02 ** 2)


class _FlakyBus(_FakeBus):
    """Times out on the first `fail` reads, then answers normally."""

    def __init__(self, raw_by_addr, fail):
        super().__init__(raw_by_addr)
        self._fail = fail

    def read_i2c_block_data(self, address, start, length):
        if self._fail > 0:
            self._fail -= 1
            raise TimeoutError(110, "Connection timed out")
        return super().read_i2c_block_data(address, start, length)


def test_failed_read_skips_the_sample_instead_of_crashing(node):
    captured = _capture_publisher(node)
    node._bus = _FlakyBus({0x20: int(1.0 / _QUA_LSB_PER_UNIT)}, fail=2)

    node._tick()
    node._tick()
    assert captured == [] and node._read_failures == 2

    node._tick()
    assert len(captured) == 1 and node._read_failures == 0


# ---- startup gate and glitch filter ----

class _Clock:
    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t

    def sleep(self, _):
        pass


def _quat_yaw(yaw_rad):
    import math
    return {0x20: int(math.cos(yaw_rad / 2) / _QUA_LSB_PER_UNIT), 0x26: int(math.sin(yaw_rad / 2) / _QUA_LSB_PER_UNIT)}


def test_placeholder_quaternion_is_held_back_until_fusion_runs(node, monkeypatch):
    import math
    clock = _Clock()
    monkeypatch.setattr("bot_imu.bno055_node.time", clock)
    captured = []
    node._publisher.publish = lambda msg: captured.append(msg)
    node._init_time, node._fusion_ready = clock.t, False
    node._bus = _FakeBus({0x20: int(1.0 / _QUA_LSB_PER_UNIT)})       # identity placeholder
    for _ in range(10):
        clock.t += 0.02
        node._tick()
    assert captured == []                                             # held back
    node._bus = _FakeBus(_quat_yaw(math.radians(131.9)))              # fusion's real heading
    clock.t += 0.02
    node._tick()
    assert len(captured) == 1                                         # the EKF's zero is the real heading


def test_all_zero_quaternion_is_a_placeholder_too(node, monkeypatch):
    clock = _Clock()
    monkeypatch.setattr("bot_imu.bno055_node.time", clock)
    captured = []
    node._publisher.publish = lambda msg: captured.append(msg)
    node._init_time, node._fusion_ready = clock.t, False
    node._bus = _FakeBus({})
    clock.t += 0.02
    node._tick()
    assert captured == []


def test_gate_gives_up_after_its_timeout(node, monkeypatch):
    clock = _Clock()
    monkeypatch.setattr("bot_imu.bno055_node.time", clock)
    captured = []
    node._publisher.publish = lambda msg: captured.append(msg)
    node._init_time, node._fusion_ready = clock.t, False
    node._bus = _FakeBus({0x20: int(1.0 / _QUA_LSB_PER_UNIT)})       # genuinely at heading 0
    clock.t += 2.5
    node._tick()
    assert len(captured) == 1


def test_one_sample_heading_glitch_is_skipped(node, monkeypatch):
    import math
    clock = _Clock()
    monkeypatch.setattr("bot_imu.bno055_node.time", clock)
    captured = _capture_publisher(node)
    for yaw in (10, 10, 115, 10, 10):                                 # +105 deg for one read, gyro 0
        node._bus = _FakeBus(_quat_yaw(math.radians(yaw)))
        clock.t += 0.02
        node._tick()
    assert len(captured) == 4                                         # the 115 sample was dropped


def test_persistent_heading_jump_is_accepted(node, monkeypatch):
    import math
    clock = _Clock()
    monkeypatch.setattr("bot_imu.bno055_node.time", clock)
    captured = _capture_publisher(node)
    for yaw in [0] + [90] * 8:                                        # e.g. the chip really reset
        node._bus = _FakeBus(_quat_yaw(math.radians(yaw)))
        clock.t += 0.02
        node._tick()
    assert len(captured) == 1 + 3                                     # 5 skipped, then accepted


def test_real_turn_backed_by_the_gyro_passes(node, monkeypatch):
    import math
    clock = _Clock()
    monkeypatch.setattr("bot_imu.bno055_node.time", clock)
    captured = _capture_publisher(node)
    yaw = 0.0
    for _ in range(10):                                               # 2 rad/s turn
        regs = _quat_yaw(yaw)
        regs[0x18] = int(2.0 * 900)                                   # gyro z, 900 LSB per rad/s
        node._bus = _FakeBus(regs)
        clock.t += 0.02
        node._tick()
        yaw += 2.0 * 0.02
    assert len(captured) == 10
