"""Two Touch devices -> F407. Dry run does not open the serial port."""

import argparse
from contextlib import contextmanager
import math
import sys
import time

from bridge import rate_hz, slew
import serial
import touch_hd as touch


@contextmanager
def open_pair(serials):
    hd = touch._hd
    devices = []
    scheduler_started = False
    try:
        for expected in serials:
            device = touch.TouchDevice(expected)
            handle = hd.hdInitDevice(expected.encode("ascii"))
            if handle != touch.HD_INVALID_HANDLE and all(d.handle != handle for d in devices):
                device.handle = handle
                devices.append(device)
            touch._check(f"opening {expected}")
            if device.handle == touch.HD_INVALID_HANDLE:
                raise RuntimeError(f"{expected}: invalid or duplicate handle")
            hd.hdMakeCurrentDevice(handle)
            actual = device.serial_number.strip()
            touch._check(f"reading serial number for {expected}")
            if actual != expected:
                raise RuntimeError(f"expected serial {expected}, got {actual!r}")

        hd.hdStartScheduler()
        scheduler_started = True
        touch._check("starting shared scheduler")
        for device in devices:
            device._wait_until_ready()
            if device.calibration_status() != "ok":
                raise RuntimeError(f"{device.config_name}: calibration not ready")
        yield devices
    finally:
        if scheduler_started:
            hd.hdStopScheduler()
        for device in reversed(devices):
            device.close()


class HandControl:
    def __init__(self, invert_linear=True, invert_rotary=False):
        self.sign = (-1 if invert_linear else 1, -1 if invert_rotary else 1)
        self.needs_release = True
        self.mode = None
        self.origin = self.hz = 0.0

    def update(self, state, dt):
        z, yaw = state.position[2], math.degrees(state.gimbal_angles[0])
        if not math.isfinite(z) or not math.isfinite(yaw):
            raise RuntimeError("invalid Touch sample")
        b1, b2 = state.button(1), state.button(2)
        if not b1 and not b2:
            self.needs_release = False
            self.mode = None
            return "0", "0"
        mode = 0 if b1 else 1
        if (b1 and b2) or (self.mode is not None and self.mode != mode):
            self.needs_release = True
        if self.needs_release:
            self.mode = None
            # The controller must not interpret both buttons down as a release.
            return "A", "0"
        value = (z, yaw)[mode]
        if self.mode is None:
            self.mode, self.origin, self.hz = mode, value, 0.0
        offset = value - self.origin
        if mode == 1:
            offset = (offset + 180.0) % 360.0 - 180.0
        target = rate_hz(offset, (2.0, 1.5)[mode], (40.0, 15.0)[mode],
                         (1600, 400)[mode], self.sign[mode])
        self.hz = slew(self.hz, target, (3000.0, 1200.0)[mode], min(dt, 0.05))
        hz = round(self.hz)
        if not hz:
            return "A", "0"
        return (str(hz), "0") if mode == 0 else ("0", str(hz))


def exchange(port, command, valid):
    port.write((command + "\r\n").encode("ascii"))
    reply = port.readline().decode("ascii", "replace").strip()
    if reply not in valid:
        raise RuntimeError(f"{command}: unexpected reply {reply or '(timeout)'}")


@contextmanager
def board_connection(port_name, live):
    if not live:
        yield None
        return
    with serial.Serial(baudrate=115200, timeout=0.12, write_timeout=0.12) as port:
        port.port = port_name
        port.dtr = port.rts = False
        port.open()
        time.sleep(1.2)
        port.reset_input_buffer()
        exchange(port, "DUAL?", {"OK DUAL V1"})
        try:
            exchange(port, "DUAL 0 0 0 0", {"OK DUAL 00"})
            yield port
        finally:
            try:
                port.write(b"DUAL 0 0 0 0\r\n")
            except (OSError, serial.SerialException):
                pass  # Firmware times out if the release cannot be delivered.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--a", default="22119000410", help="Touch serial for M1/M2")
    parser.add_argument("--b", default="22109000332", help="Touch serial for M4/M5")
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl+C")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--invert-b-linear", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--invert-b-rotary", action=argparse.BooleanOptionalAction, default=False)
    args = parser.parse_args()
    if args.a == args.b or not all(s.isascii() and s.isdecimal() for s in (args.a, args.b)):
        parser.error("--a and --b must be distinct numeric hardware serials")
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and nonnegative")

    controls = HandControl(), HandControl(args.invert_b_linear, args.invert_b_rotary)
    try:
        with open_pair((args.a, args.b)) as devices, board_connection(args.port, args.live) as port:
            print(f"{'LIVE' if port else 'DRY RUN (no serial)'}: "
                  f"A {args.a} -> M1/M2 | B {args.b} -> M4/M5", flush=True)
            print("Release both buttons first. Button 1: Z; button 2: wrist.", flush=True)
            start = last = time.monotonic()
            next_calibration = start + 0.5
            previous = None
            while True:
                now = time.monotonic()
                if args.seconds and now - start >= args.seconds:
                    break
                if port and now - last >= 0.2:
                    raise RuntimeError("control loop stalled")
                if now >= next_calibration:
                    if any(d.calibration_status() != "ok" for d in devices):
                        raise RuntimeError("Touch lost calibration")
                    next_calibration = now + 0.5
                values = [control.update(device.read(), now - last)
                          for device, control in zip(devices, controls)]
                last = now
                if port:
                    if time.monotonic() - now >= 0.15:
                        raise RuntimeError("Touch sampling stalled")
                    exchange(port, "DUAL " + " ".join(values[0] + values[1]),
                             {"OK DUAL 00", "OK DUAL 10", "OK DUAL 01", "OK DUAL 11"})
                if values != previous:
                    print(f"A (M1 M2): {' '.join(values[0])} | "
                          f"B (M4 M5): {' '.join(values[1])}", flush=True)
                    previous = values
                time.sleep(max(0.0, 0.05 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        return 130
    except (touch.HDError, RuntimeError, OSError) as exc:
        print(f"Stopped: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
