#!/usr/bin/env python3
"""Read-only STS3215 raw position helper; never writes motor registers."""
import argparse
from servo_bus import load_sdk
import time
import config
from kinematics import NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch', action='store_true')
    parser.add_argument('--port', default=config.PORT, help='Serial device (default: config.PORT)')
    parser.add_argument('--baudrate', type=int, default=config.BAUDRATE)
    args = parser.parse_args()
    sdk = load_sdk(config.settings())
    port = sdk.PortHandler(args.port)
    try:
        if not port.setBaudRate(args.baudrate):
            raise OSError('Unable to open configured serial port')
        handler = sdk.sms_sts(port)
        while True:
            raw = {}
            for name in NAMES:
                value, result, error = handler.ReadPos(config.SERVO_IDS[name])
                if result != sdk.COMM_SUCCESS or error:
                    detail = handler.getTxRxResult(result) if result != sdk.COMM_SUCCESS else handler.getRxPacketError(error)
                    raise OSError(f'{name} (ID {config.SERVO_IDS[name]}): {detail}; result={result}, error={error}')
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
