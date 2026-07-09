from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

# --- Spec (matches test_limo.m, main.m, README) ---
AMPLITUDE = 0.75
PERIOD = 40.0  # s — one full figure-8 cycle
OBSTACLE_CENTER = np.array([-0.20, 0.425])
OBSTACLE_RADIUS = 0.15
OBSTACLE_INFLUENCE = 0.50
LIMO_IC = np.array([0.40, -0.25])  # enunciado — centro do chassi
POI_OFFSET = 0.10  # m — PoI à frente do LIMO (yaw=0 na CI)


def lemniscate_xy(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    phase_x = 2.0 * np.pi * t / PERIOD
    phase_y = 4.0 * np.pi * t / PERIOD
    x = AMPLITUDE * np.sin(phase_x)
    y = AMPLITUDE * np.sin(phase_y)
    return x, y


def lemniscate_velocity(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    phase_x = 2.0 * np.pi * t / PERIOD
    phase_y = 4.0 * np.pi * t / PERIOD
    vx = AMPLITUDE * (2.0 * np.pi / PERIOD) * np.cos(phase_x)
    vy = AMPLITUDE * (4.0 * np.pi / PERIOD) * np.cos(phase_y)
    return vx, vy


def distance_to_obstacle(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return np.hypot(x - OBSTACLE_CENTER[0], y - OBSTACLE_CENTER[1])


def apply_obstacle_null_space_xy(vel_xy: np.ndarray, poi_xy: np.ndarray) -> np.ndarray:
    """Same law as test_limo.m / simulator.py (PoI XY subspace)."""
    offset = poi_xy - OBSTACLE_CENTER
    distance = float(np.linalg.norm(offset))
    if distance >= OBSTACLE_INFLUENCE or distance <= 1e-6:
        return vel_xy.copy()

    direction = offset / distance
    j_obs_pinv = direction.reshape(2, 1)

    clearance = distance - OBSTACLE_RADIUS
    if clearance <= 0.0:
        obstacle_rate = 0.8
    else:
        obstacle_rate = 0.4 * (
            1.0 / clearance - 1.0 / (OBSTACLE_INFLUENCE - OBSTACLE_RADIUS)
        )

    primary = (j_obs_pinv * obstacle_rate).reshape(2)
    null_projector = np.eye(2) - j_obs_pinv @ direction.reshape(1, 2)
    return primary + null_projector @ vel_xy


def simulate_poi_with_avoidance(
    t: np.ndarray, kq: float, lq: float, dt: float
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate PoI tracking + null-space avoidance (kinematic preview)."""
    poi = LIMO_IC + np.array([POI_OFFSET, 0.0])  # yaw ≈ 0 at IC
    xs = np.empty_like(t)
    ys = np.empty_like(t)
    for i, ti in enumerate(t):
        xs[i], ys[i] = poi
        xref, yref = lemniscate_xy(np.array([ti]))
        vxref, vyref = lemniscate_velocity(np.array([ti]))
        err = np.array([xref[0] - poi[0], yref[0] - poi[1]])
        vel = np.array([vxref[0], vyref[0]]) + lq * np.tanh((kq / lq) * err)
        vel = apply_obstacle_null_space_xy(vel, poi)
        if i + 1 < len(t):
            poi = poi + vel * (t[i + 1] - ti)
    return xs, ys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot base lemniscate trajectory and obstacle (UFES Robótica Móvel 2026/1)",
    )
    parser.add_argument(
        "--t-final",
        type=float,
        default=80.0,
        help="Duration to plot (s); 80 = two 40 s periods",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=1.0 / 30.0,
        help="Sample period (s)",
    )
    parser.add_argument(
        "--kq",
        type=float,
        default=1.2,
        help="Proportional gain (for avoidance preview curve)",
    )
    parser.add_argument(
        "--lq",
        type=float,
        default=0.8,
        help="tanh saturation (for avoidance preview curve)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("results"),
        help="Where to save PNG",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save only; do not open interactive window",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Skip kinematic preview with null-space avoidance",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    t = np.arange(0.0, args.t_final + args.dt / 2, args.dt)
    x_ref, y_ref = lemniscate_xy(t)
    dist = distance_to_obstacle(x_ref, y_ref)

    min_idx = int(np.argmin(dist))
    in_influence = dist < OBSTACLE_INFLUENCE
    in_collision = dist < OBSTACLE_RADIUS

    print("=== Trajetória de referência (lemniscata de Bernoulli) ===")
    print(
        f"  x_d = {AMPLITUDE}·sin(2πt/{PERIOD}),  y_d = {AMPLITUDE}·sin(4πt/{PERIOD})"
    )
    print(f"  Amostras: {len(t)} pontos, t ∈ [0, {args.t_final}] s")
    print()
    print("=== Obstáculo ===")
    print(f"  Centro: ({OBSTACLE_CENTER[0]:.2f}, {OBSTACLE_CENTER[1]:.2f}) m")
    print(f"  Raio: {OBSTACLE_RADIUS:.2f} m | Influência: {OBSTACLE_INFLUENCE:.2f} m")
    print()
    print("=== Referência vs obstáculo (PoI desejado) ===")
    print(
        f"  Distância mínima ao centro: {dist[min_idx]:.3f} m em t={t[min_idx]:.1f} s"
    )
    print(f"  Ponto mais próximo: ({x_ref[min_idx]:+.3f}, {y_ref[min_idx]:+.3f}) m")
    print(
        f"  Entra na zona de influência (0.5 m)? {'Sim' if np.any(in_influence) else 'Não'}"
    )
    print(
        f"  Cruza o disco do obstáculo (0.15 m)? {'Sim' if np.any(in_collision) else 'Não'}"
    )
    if np.any(in_influence):
        t_enter = t[in_influence][0]
        t_exit = t[in_influence][-1]
        print(f"  Na influência aprox.: t ∈ [{t_enter:.1f}, {t_exit:.1f}] s")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.output_dir / "reference_trajectory_obstacle.png"

    fig, axes = plt.subplots(1, 2 if not args.no_preview else 1, figsize=(14, 6))
    if args.no_preview:
        axes = [axes]

    def draw_obstacle(ax: plt.Axes) -> None:
        influence = Circle(
            OBSTACLE_CENTER,
            OBSTACLE_INFLUENCE,
            fill=False,
            linestyle="--",
            edgecolor="#78716c",
            linewidth=1.5,
            alpha=0.7,
            label=f"Influência ({OBSTACLE_INFLUENCE} m)",
        )
        obstacle = Circle(
            OBSTACLE_CENTER,
            OBSTACLE_RADIUS,
            facecolor="#78716c",
            edgecolor="#57534e",
            alpha=0.65,
            label=f"Obstáculo (R={OBSTACLE_RADIUS} m)",
        )
        ax.add_patch(influence)
        ax.add_patch(obstacle)
        ax.plot(
            OBSTACLE_CENTER[0],
            OBSTACLE_CENTER[1],
            "k+",
            markersize=10,
            markeredgewidth=2,
            label="Centro obstáculo",
        )

    # --- Left: reference path ---
    ax_ref = axes[0]
    draw_obstacle(ax_ref)
    ax_ref.plot(x_ref, y_ref, "r--", linewidth=2, label="Referência (lemniscata)")
    ax_ref.plot(x_ref[0], y_ref[0], "go", markersize=9, label="Início (t=0)")
    ax_ref.plot(
        x_ref[-1], y_ref[-1], "ks", markersize=8, label=f"Fim (t={args.t_final}s)"
    )
    ax_ref.plot(
        x_ref[min_idx],
        y_ref[min_idx],
        "m*",
        markersize=14,
        label=f"Mais próximo (d={dist[min_idx]:.2f} m)",
    )
    poi_ic = LIMO_IC + np.array([POI_OFFSET, 0.0])
    ax_ref.plot(LIMO_IC[0], LIMO_IC[1], "b^", markersize=9, label="CI LIMO (chassi)")
    ax_ref.plot(poi_ic[0], poi_ic[1], "bv", markersize=9, label="CI PoI (approx.)")
    ax_ref.set_xlabel("X (m)")
    ax_ref.set_ylabel("Y (m)")
    ax_ref.set_title("Trajetória base + obstáculo\n(referência do enunciado)")
    ax_ref.axis("equal")
    ax_ref.grid(True, alpha=0.35)
    ax_ref.legend(loc="upper right", fontsize=8)

    # --- Right: kinematic preview with null-space ---
    if not args.no_preview:
        x_poi, y_poi = simulate_poi_with_avoidance(t, args.kq, args.lq, args.dt)
        ax_prev = axes[1]
        draw_obstacle(ax_prev)
        ax_prev.plot(x_ref, y_ref, "r--", linewidth=1.2, alpha=0.5, label="Referência")
        ax_prev.plot(
            x_poi,
            y_poi,
            "b-",
            linewidth=1.8,
            label="Preview PoI (ctrl + null-space)",
        )
        ax_prev.plot(x_poi[0], y_poi[1], "go", markersize=8)
        ax_prev.plot(x_poi[-1], y_poi[-1], "ks", markersize=7)
        ax_prev.set_xlabel("X (m)")
        ax_prev.set_ylabel("Y (m)")
        ax_prev.set_title(
            f"Preview cinemático PoI\n(kq={args.kq}, lq={args.lq}; sem dinâmica LIMO)"
        )
        ax_prev.axis("equal")
        ax_prev.grid(True, alpha=0.35)
        ax_prev.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print()
    print(f"Figura salva em: {out_path.resolve()}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
