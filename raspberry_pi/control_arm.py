#!/usr/bin/env python3
"""Local Pi keyboard control. Dry-run by default; --live explicitly opens bus."""
import argparse
import json
import math
import select
import signal
import time
import numpy as np
import config
from kinematics import Robot, NAMES
from controller import Controller
from servo_bus import ServoBus, calibrated_joints


def run_keys(args):
    from evdev import InputDevice, ecodes, list_devices
    if args.list_keyboards:
        for path in list_devices():
            with InputDevice(path) as dev:
                if ecodes.KEY_W in dev.capabilities().get(ecodes.EV_KEY, []):
                    print(path, dev.name)
        return
    if not args.keyboard:
        raise ValueError('Select a local keyboard with --list-keyboards, then --keyboard /dev/input/eventN')
    robot = Robot()
    settings, bus = config.settings(), None
    settings.update(port=args.port, baudrate=args.baudrate)
    maps = calibrated_joints(settings, robot) if args.live else None
    rates = [m.max_deg_s for m in maps] if maps else list(config.MAX_JOINT_DEG_PER_SECOND[n] for n in NAMES)
    controller = Controller(robot, rates, config.SPEED_SCALE)
    if not math.isfinite(config.MAX_TRACKING_ERROR_DEG) or config.MAX_TRACKING_ERROR_DEG <= 0:
        raise ValueError('Tracking threshold must be positive')
    if not math.isfinite(config.LOOP_TIMEOUT_SECONDS) or config.LOOP_TIMEOUT_SECONDS < .08:
        raise ValueError('Loop timeout must be at least 0.08 seconds')
    codes = {getattr(ecodes, 'KEY_'+key.upper()): key for key in
             ('w', 's', 'a', 'd', 'j', 'l', 'up', 'down', 'left', 'right', 'e', 'q')}
    codes[ecodes.KEY_ESC] = 'escape'
    modifier_codes = {ecodes.KEY_LEFTCTRL, ecodes.KEY_RIGHTCTRL, ecodes.KEY_LEFTALT,
                      ecodes.KEY_RIGHTALT, ecodes.KEY_LEFTMETA, ecodes.KEY_RIGHTMETA}
    device, grabbed, quit_requested = None, False, False

    def stop_signal(_signum, _frame):
        nonlocal quit_requested
        quit_requested = True
    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)

    try:
        device = InputDevice(args.keyboard)
        if ecodes.KEY_W not in device.capabilities().get(ecodes.EV_KEY, []):
            raise ValueError('Selected input device is not a keyboard')
        device.grab(); grabbed = True
        if args.live:
            bus = ServoBus(settings, maps)
            robot.q = robot.from_display(bus.read_degrees())
        print('LIVE' if bus else 'DRY RUN - no serial connection or motor commands')
        print('E enable + level | W/S Z | arrows yaw/reach | A/D pitch | J/L open/close | Esc pause | Q quit')
        print('Keyboard is exclusive while running. Escape/Q hold the arm; they do not remove motor power.')
        next_tick, previous_tick, last_print = time.monotonic(), time.monotonic(), 0
        while not quit_requested:
            readable, _, _ = select.select([device.fd], [], [], max(0, next_tick-time.monotonic()))
            if readable:
                for event in device.read():
                    if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_DROPPED:
                        raise OSError('Keyboard events lost; stopping')
                    if event.type != ecodes.EV_KEY or event.value == 2:
                        continue  # OS autorepeat must not retrigger enable or gripper.
                    key = codes.get(event.code)
                    if event.code in modifier_codes and event.value:
                        controller.stop()
                        if bus: bus.hold_measured()
                    if key == 'q' and event.value:
                        quit_requested = True; controller.stop(); break
                    if not key:
                        continue
                    if key == 'e' and event.value and not controller.enabled and bus:
                        # No jump to a fabricated initial raw value.
                        robot.q = robot.from_display(bus.read_degrees(check_mode=True))
                    was_enabled = controller.enabled
                    controller.key(key, event.value == 1)
                    if was_enabled and not controller.enabled and bus:
                        bus.hold_measured()
                        robot.q = robot.from_display(bus.read_degrees())
            if quit_requested:
                break
            now = time.monotonic()
            if now < next_tick:
                continue
            if controller.enabled and now-previous_tick > config.LOOP_TIMEOUT_SECONDS:
                raise TimeoutError('Control loop exceeded timeout; stopping')
            previous_tick = now
            # Reconcile physical key state too, protecting against missed releases.
            active = set(device.active_keys())
            controller.held.intersection_update(codes[c] for c in active if c in codes)
            if controller.enabled and active & modifier_codes:
                controller.stop()
                if bus: bus.hold_measured()
            if controller.enabled and bus:
                measured = bus.read_degrees(check_mode=True)
                if np.max(np.abs(measured-robot.display())) > config.MAX_TRACKING_ERROR_DEG:
                    raise RuntimeError('Servo tracking error exceeds limit; stopping')
            changed = controller.tick(.08)
            # A slow IK solve must not emit a stale command after the watchdog.
            if controller.enabled and time.monotonic()-now > config.LOOP_TIMEOUT_SECONDS:
                raise TimeoutError('IK/feedback exceeded timeout; stopping')
            if changed and bus:
                bus.write_degrees(robot.display())
            if time.monotonic()-last_print >= .4:
                print(controller.status, '| deg', np.round(robot.display(), 1).tolist(),
                      '| XYZ m', np.round(robot.pose()[0], 4).tolist(), flush=True)
                last_print = time.monotonic()
            # No burst of catch-up commands after a delayed tick.
            next_tick = max(now+.08, time.monotonic())
    finally:
        controller.stop()
        if bus:
            try:
                bus.hold_measured()
            except Exception as ex:
                print('Could not confirm stop over serial:', ex, flush=True)
            finally:
                bus.close()
        if device:
            if grabbed:
                try: device.ungrab()
                except OSError: pass
            device.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true', help='Use calibrated STS3215 hardware; default is dry-run')
    parser.add_argument('--keyboard', help='Local Linux keyboard event device')
    parser.add_argument('--port', default=config.PORT, help='Serial device (default: config.PORT)')
    parser.add_argument('--baudrate', type=int, default=config.BAUDRATE)
    parser.add_argument('--list-keyboards', action='store_true')
    parser.add_argument('--demo', action='store_true', help='Finite offline test; no keyboard/serial access')
    args = parser.parse_args()
    if args.demo:
        if args.live:
            parser.error('--demo cannot be combined with --live')
        robot = Robot(); controller = Controller(robot)
        controller.enable()
        for _ in range(1000):
            controller.tick()
            if controller.level_target is None: break
        for key in ('w', 's', 'right', 'left', 'up', 'down', 'a', 'd', 'j', 'l'):
            controller.key(key, True)
            for _ in range(10): controller.tick()
            controller.key(key, False)
        print(json.dumps(dict(mode='dry-run', degrees=robot.display().tolist(), position=robot.pose()[0].tolist())))
    else:
        run_keys(args)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError) as ex:
        raise SystemExit(str(ex))
