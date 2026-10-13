#!/usr/bin/env python3
"""Read-only STS3215 raw position helper; never writes motor registers."""
import argparse
import importlib
import time
import config
from kinematics import NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch', action='store_true')
    args = parser.parse_args()
    sdk = importlib.import_module(config.SDK_MODULE)
    port = sdk.PortHandler(config.PORT)
    try:
        if not port.setBaudRate(config.BAUDRATE):
            raise OSError('Unable to open configured serial port')
        handler = sdk.sms_sts(port)
        while True:
            raw = {}
            for name in NAMES:
                value, result, error = handler.ReadPos(config.SERVO_IDS[name])
                if result != sdk.COMM_SUCCESS or error:
                    raise OSError(f'{name}: communication={result}, servo error={error}')
                raw[name] = value
            print(raw, flush=True)
            if not args.watch: break
            time.sleep(.5)
    finally:
        if getattr(port, 'is_open', False): port.closePort()


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: pass
    except (OSError, ImportError) as ex: raise SystemExit(str(ex))
