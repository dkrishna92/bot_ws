#!/usr/bin/env python3
"""Log e-stop relay dropouts (lost heartbeats) from the Pi, with no firmware changes.

When the e-stop Nano de-energizes its relay, the Pololu G2 loses motor
power and pulls its FLT pins low (under-voltage). This keeps the G2 awake
with PWM at zero -- THE MOTORS NEVER RUN -- and watches FLT at ~1 kHz,
printing each dropout with its duration, plus a summary every 10 s. Walk
the kill-switch transmitter out to course range and watch where it starts
dropping, then change one thing (antenna placement, module power, air
rate) and walk it again.

A dropout is either a missed-heartbeat timeout (link loss) or the kill
switch being pressed -- the Pi can't tell which; you know when you press it.

First run: press the kill switch once and check it logs a dropout. If it
doesn't, FLT doesn't follow motor power on this build and this tool can't
see the relay.

Stop any ROS launch first (it can't share the motor GPIO). Ctrl-C to end.

Usage:
    scripts/estop_link_log.py
    scripts/estop_link_log.py --summary-every 30
"""
import argparse
import time

import lgpio

import drive_lib as dl


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--summary-every", type=float, default=10.0, help="seconds between summaries (default 10)")
    args = ap.parse_args()

    dl.require_hardware_free()
    h = lgpio.gpiochip_open(dl.GPIO_CHIP)
    try:
        for ch in dl.CHANNELS.values():
            lgpio.gpio_claim_output(h, ch["pwm"], 0)   # PWM stays 0: motors never run
            lgpio.gpio_claim_output(h, ch["dir"], 0)
            lgpio.gpio_claim_output(h, ch["sleep"], 1)  # awake, so FLT reflects the supply
        for pin in dl.FAULT_PINS.values():
            lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)
        time.sleep(0.01)  # the G2's ~1 ms wake-up FLT pulse

        def power_ok():
            return all(lgpio.gpio_read(h, pin) == 1 for pin in dl.FAULT_PINS.values())

        start = time.monotonic()
        up = power_ok()
        print(f"{0:8.2f}s  motor power {'ON (relay closed)' if up else 'OFF (relay open / no heartbeat)'}"
              " -- logging dropouts, Ctrl-C to stop", flush=True)
        changed_at = start
        drops = []  # durations of completed dropouts (s)
        window_drops = 0
        last_summary = start
        while True:
            now = time.monotonic()
            ok = power_ok()
            if ok != up:
                if ok:
                    dur = now - changed_at
                    drops.append(dur)
                    print(f"{now - start:8.2f}s  restored after {dur * 1000:6.0f} ms", flush=True)
                else:
                    window_drops += 1
                    print(f"{now - start:8.2f}s  DROPOUT (relay opened)", flush=True)
                up, changed_at = ok, now
            if now - last_summary >= args.summary_every:
                down_now = "" if up else f", currently DOWN for {now - changed_at:.1f} s"
                print(f"{now - start:8.2f}s  -- last {args.summary_every:.0f} s: {window_drops} dropout(s); "
                      f"total {len(drops) + (0 if up else 1)}{down_now}", flush=True)
                window_drops, last_summary = 0, now
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        for ch in dl.CHANNELS.values():
            lgpio.tx_pwm(h, ch["pwm"], dl.PWM_HZ, 0)
            lgpio.gpio_write(h, ch["sleep"], 0)
        lgpio.gpiochip_close(h)
    if drops:
        print(f"\n{len(drops)} completed dropout(s): shortest {min(drops) * 1000:.0f} ms, "
              f"longest {max(drops) * 1000:.0f} ms, total {sum(drops):.1f} s")
    else:
        print("\nno completed dropouts")


if __name__ == "__main__":
    main()
