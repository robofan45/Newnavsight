"""Command line entry point: ``navsight`` or ``python -m navsight``."""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path

from navsight import __version__
from navsight.config import NavSightConfig, dump_config, expand_path, load_config


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="navsight", description="NavSight v2 smart cane for people with visual impairment"
    )
    parser.add_argument("--config", type=Path, help="JSON file with settings to override")
    parser.add_argument(
        "--sim", action="store_true", help="run the scripted simulator instead of the hardware"
    )
    parser.add_argument("--duration", type=float, help="stop after this many seconds")
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="simulator speed: 1 = real time (default), 0 = as fast as possible",
    )
    parser.add_argument(
        "--battery-full",
        action="store_true",
        help="mark the battery as fully charged (run after charging) and exit",
    )
    parser.add_argument(
        "--print-config", action="store_true", help="print the effective settings as JSON and exit"
    )
    parser.add_argument("--log-level", help="DEBUG, INFO, WARNING or ERROR")
    parser.add_argument("--version", action="version", version=f"navsight {__version__}")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        config = load_config(args.config)
    except (OSError, ValueError) as error:
        print(f"navsight: {error}", file=sys.stderr)
        return 2
    level_name = (args.log_level or config.log_level).upper()
    level = logging.getLevelName(level_name)
    if not isinstance(level, int):
        print(f"navsight: unknown log level {level_name!r}", file=sys.stderr)
        return 2
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    if args.print_config:
        print(dump_config(config))
        return 0
    if args.battery_full:
        return mark_battery_full(config)
    if args.sim or config.simulate:
        from navsight.sim import run_simulation

        run_simulation(config, duration_s=args.duration, speed=max(0.0, args.speed))
        return 0
    return run_hardware(config, args.duration)


def mark_battery_full(config: NavSightConfig) -> int:
    from navsight.hardware.power_monitor import StateOfChargeEstimator

    path = expand_path(config.power.state_file)
    estimator = StateOfChargeEstimator(config.power, path)
    estimator.reset_full()
    estimator.save()
    print(f"Battery marked as fully charged ({path}).")
    return 0


def run_hardware(config: NavSightConfig, duration_s: float | None) -> int:
    from navsight.app import NavSightApp, build_devices

    app = NavSightApp(config, build_devices(config))
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, lambda _signum, _frame: app.request_stop())
    try:
        app.start()
        app.run(duration_s)
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
