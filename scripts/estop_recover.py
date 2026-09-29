#!/usr/bin/env python3
"""Recover the e-stop Nano ESP32 (ESP32-S3) stuck in ROM download mode (303a:1001), in software.

Why it gets stuck (found 2026-09-29): on a full battery power-up the
Nano's supply ramps slowly and the chip leaves reset before its strapping
pins read HIGH -- GPIO_STRAP_REG reads 0x00000000, so GPIO0 is latched LOW
and the ROM waits for a download instead of booting the firmware. A USB
replug (fast ramp) or pressing reset (supply already stable) boots fine.
Needs the Nano on the Pi's USB. The relay stays de-energized (motors cut)
the whole time -- the firmware only energizes it on a clean heartbeat.

Talks the ROM loader's SLIP protocol directly (no esptool needed):
  1. SYNC
  2. read GPIO_STRAP_REG (boot-pin levels latched at reset) and
     RTC_CNTL_OPTION1_REG (FORCE_DOWNLOAD_BOOT flag, set by the Arduino
     core's "reboot into bootloader" path)
  3. clear FORCE_DOWNLOAD_BOOT
  4. RTC watchdog reset -> normal boot into the app (same as esptool's
     watchdog_reset for ESP32-S3)
DTR/RTS are held low so opening the port doesn't itself reset the chip.
"""
import argparse
import glob
import struct
import sys
import time

import serial

GPIO_STRAP_REG = 0x60004038
RTC_CNTL_BASE = 0x60008000
RTC_CNTL_OPTION1_REG = RTC_CNTL_BASE + 0x12C
RTC_CNTL_WDTCONFIG0_REG = RTC_CNTL_BASE + 0x98
RTC_CNTL_WDTCONFIG1_REG = RTC_CNTL_BASE + 0x9C
RTC_CNTL_WDTWPROTECT_REG = RTC_CNTL_BASE + 0xB0
RTC_CNTL_WDT_WKEY = 0x50D83AA1

OP_WRITE_REG, OP_READ_REG, OP_SYNC = 0x09, 0x0A, 0x08


def slip(data):
    return b"\xc0" + data.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc") + b"\xc0"


def read_frame(s, timeout=1.0):
    end = time.time() + timeout
    buf, inside = b"", False
    while time.time() < end:
        c = s.read(1)
        if not c:
            continue
        if c == b"\xc0":
            if inside and buf:
                return buf.replace(b"\xdb\xdc", b"\xc0").replace(b"\xdb\xdd", b"\xdb")
            inside, buf = True, b""
        elif inside:
            buf += c
    return None


def command(s, op, data=b"", timeout=1.0):
    s.write(slip(struct.pack("<BBHI", 0, op, len(data), 0) + data))
    end = time.time() + timeout
    while time.time() < end:
        f = read_frame(s, end - time.time())
        if f and len(f) >= 8 and f[0] == 1 and f[1] == op:
            return struct.unpack("<I", f[4:8])[0], f[8:]
    raise RuntimeError(f"no reply to op 0x{op:02x}")


def read_reg(s, addr):
    return command(s, OP_READ_REG, struct.pack("<I", addr))[0]


def write_reg(s, addr, value, mask=0xFFFFFFFF):
    command(s, OP_WRITE_REG, struct.pack("<IIII", addr, value, mask, 0))


parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument(
    "--wait", type=float, default=0.0, metavar="SEC",
    help="wait this long first and only act if the chip is STILL in download "
         "mode -- a normal boot also shows 303a:1001 for ~0.6 s (used by "
         "the estop-recover.service udev trigger)")
args = parser.parse_args()


def stuck_ports():
    return glob.glob("/dev/serial/by-id/usb-Espressif_USB_JTAG*")


if args.wait:
    time.sleep(args.wait)
ports = stuck_ports()
if not ports:
    print("no 303a:1001 device -- not in the stuck state")
    sys.exit(0)

s = serial.Serial()
s.port, s.baudrate, s.timeout = ports[0], 115200, 0.05
s.dtr = s.rts = False
s.open()
s.reset_input_buffer()

sync = b"\x07\x07\x12\x20" + b"\x55" * 32
for attempt in range(10):
    try:
        command(s, OP_SYNC, sync, timeout=0.3)
        break
    except RuntimeError:
        pass
else:
    sys.exit("ROM loader did not answer SYNC")
time.sleep(0.1)
s.reset_input_buffer()  # SYNC gets several replies

strap = read_reg(s, GPIO_STRAP_REG)
opt1 = read_reg(s, RTC_CNTL_OPTION1_REG)
print(f"GPIO_STRAP_REG       = 0x{strap:08x}  (GPIO0 level at reset: {'HIGH (SPI boot)' if strap & (1 << 3) else 'LOW (download)'})")
print(f"RTC_CNTL_OPTION1_REG = 0x{opt1:08x}  (FORCE_DOWNLOAD_BOOT: {'SET' if opt1 & 1 else 'clear'})")

if opt1 & 1:
    write_reg(s, RTC_CNTL_OPTION1_REG, 0, mask=1)
    print(f"cleared FORCE_DOWNLOAD_BOOT -> 0x{read_reg(s, RTC_CNTL_OPTION1_REG):08x}")

print("triggering RTC watchdog reset...")
write_reg(s, RTC_CNTL_WDTWPROTECT_REG, RTC_CNTL_WDT_WKEY)
write_reg(s, RTC_CNTL_WDTCONFIG1_REG, 5000)
try:
    write_reg(s, RTC_CNTL_WDTCONFIG0_REG, (1 << 31) | (5 << 28) | (1 << 8) | 2)
    write_reg(s, RTC_CNTL_WDTWPROTECT_REG, 0)
except (RuntimeError, serial.SerialException, OSError):
    pass  # chip may reset before replying
try:
    s.close()
except Exception:
    pass
