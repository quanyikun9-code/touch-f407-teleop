"""Read two named Touch devices without serial I/O or force output."""

import argparse
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device_a", help="first Touch configuration name")
    parser.add_argument("device_b", help="second Touch configuration name")
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error("--seconds must be greater than 0 and at most 60")
    if args.device_a.casefold() == args.device_b.casefold():
        parser.error("pass two different configuration names")

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                          "tools" / "3dtouch" / "code" / "touchpy"))
    import touch_hd as touch

    hd = touch._hd
    devices = []
    scheduler_started = False
    try:
        for name in (args.device_a, args.device_b):
            device = touch.TouchDevice(name)
            handle = hd.hdInitDevice(name.encode("utf-8"))
            if handle != touch.HD_INVALID_HANDLE and all(handle != d.handle for d in devices):
                device.handle = handle
                devices.append(device)
            touch._check(f"opening {name!r}")
            if device.handle == touch.HD_INVALID_HANDLE:
                raise RuntimeError(f"{name!r}: invalid or duplicate device handle")

        serials = []
        for label, device in zip("AB", devices):
            hd.hdMakeCurrentDevice(device.handle)
            touch._check(f"selecting device {label}")
            serials.append(device.serial_number.strip())
            touch._check(f"reading serial number for {label}")
        if not all(serials) or serials[0] == serials[1]:
            raise RuntimeError(f"two distinct device serials required, got {serials!r}")

        hd.hdStartScheduler()
        scheduler_started = True
        touch._check("starting shared scheduler")
        for label, device, serial in zip("AB", devices, serials):
            device._wait_until_ready()
            print(f"{label}: {device.config_name} serial={serial} "
                  f"calibration={device.calibration_status()}", flush=True)

        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            samples = []
            for label, device in zip("AB", devices):
                state = device.read()
                samples.append(f"{label} Z={state.position[2]:+7.2f} mm "
                               f"gimbal1={state.gimbal_angles[0]:+.3f} rad buttons={state.buttons}")
            print(" | ".join(samples), flush=True)
            time.sleep(0.2)
        return 0
    except (touch.HDError, RuntimeError) as exc:
        print(f"Probe stopped: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if scheduler_started:
            hd.hdStopScheduler()
        for device in reversed(devices):
            device.close()


if __name__ == "__main__":
    sys.exit(main())
