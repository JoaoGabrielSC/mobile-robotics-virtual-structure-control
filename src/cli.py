import argparse
from pathlib import Path

from config import SimulationConfig


def parse_cli() -> SimulationConfig:
    parser = argparse.ArgumentParser(
        description="Virtual structure formation simulator (LIMO + Bebop 2) — validated 2026",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--t_final",
        type=float,
        default=100.0,
        help="Total simulation time in seconds",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=1.0 / 30.0,
        help="Sampling period T in seconds (spec: 30 Hz)",
    )
    parser.add_argument(
        "--kq",
        type=float,
        default=1.2,
        help="Proportional gain of the formation controller",
    )
    parser.add_argument(
        "--lq",
        type=float,
        default=0.8,
        help="Hyperbolic tangent saturation limit",
    )
    parser.add_argument(
        "--kd_limo",
        type=float,
        default=4.0,
        help="LIMO dynamic compensator gain",
    )
    parser.add_argument(
        "--anim",
        action="store_true",
        help="Animate LIMO and Bebop 2 motion",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output"),
        help="Directory where plot images are saved",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save plots without opening interactive windows",
    )
    args = parser.parse_args()
    return SimulationConfig(
        t_final=args.t_final,
        dt=args.dt,
        kq=args.kq,
        lq=args.lq,
        kd_limo=args.kd_limo,
        animate=args.anim,
        output_dir=args.output_dir,
        show_plots=not args.no_show,
    )
