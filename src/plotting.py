from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from config import SimulationConfig

OBSTACLE_CENTER = (-0.2, 0.425)
OBSTACLE_RADIUS = 0.15
OBSTACLE_INFLUENCE = 0.5

FloatArray = npt.NDArray[np.floating]
Line3DArtist = Any

ROBOTS_PLOT_NAME = "robots.png"
ERRORS_PLOT_NAME = "formation_errors.png"
PLOT_DPI = 150


def _finite(value: float, default: float = 0.0) -> float:
    return value if np.isfinite(value) else default


def _limo_corners(
    x: float, y: float, yaw: float, body_length: float = 0.12
) -> tuple[FloatArray, FloatArray, FloatArray]:
    x, y, yaw = _finite(x), _finite(y), _finite(yaw)
    corners = np.array(
        [
            [-body_length, -body_length / 2],
            [body_length, -body_length / 2],
            [body_length, body_length / 2],
            [-body_length, body_length / 2],
            [-body_length, -body_length / 2],
        ]
    )
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    corners = (rotation @ corners.T).T + np.array([x, y])
    return corners[:, 0], corners[:, 1], np.zeros(len(corners))


def _update_line_3d(
    line: Line3DArtist,
    xs: FloatArray | list[float],
    ys: FloatArray | list[float],
    zs: FloatArray | list[float],
) -> None:
    line.set_data(np.asarray(xs, dtype=float), np.asarray(ys, dtype=float))
    line.set_3d_properties(np.asarray(zs, dtype=float))


def _update_point_3d(line: Line3DArtist, x: float, y: float, z: float) -> None:
    _update_line_3d(line, [_finite(x)], [_finite(y)], [_finite(z)])


def _draw_limo_3d(
    ax: Axes3D,
    x: float,
    y: float,
    yaw: float,
    poi_offset: float,
    color: str = "#2563eb",
) -> None:
    xs, ys, zs = _limo_corners(x, y, yaw)
    ax.plot(xs, ys, zs, color=color, linewidth=2)
    poi_x = _finite(x) + poi_offset * np.cos(_finite(yaw))
    poi_y = _finite(y) + poi_offset * np.sin(_finite(yaw))
    ax.plot([poi_x], [poi_y], [0.0], "o", color="#1d4ed8", markersize=6)
    ax.plot(
        [_finite(x), _finite(x) + 0.15 * np.cos(_finite(yaw))],
        [_finite(y), _finite(y) + 0.15 * np.sin(_finite(yaw))],
        [0.0, 0.0],
        color="black",
        linewidth=1.5,
    )


def _draw_bebop_3d(ax: Axes3D, x: float, y: float, z: float, color: str = "#dc2626") -> None:
    x, y, z = _finite(x), _finite(y), _finite(z)
    ax.plot([x], [y], [z], "^", color=color, markersize=10)
    ax.plot([x, x], [y, y], [0.0, z], color=color, linestyle=":", alpha=0.5)


def _axis_limits(
    hist_limo: FloatArray,
    hist_bebop: FloatArray,
    hist_qd: FloatArray,
    hist_poi_bebop: FloatArray,
    margin: float = 0.5,
) -> tuple[float, float, float, float, float]:
    all_x = np.concatenate([hist_limo[0, :], hist_bebop[0, :], hist_qd[0, :]])
    all_y = np.concatenate([hist_limo[1, :], hist_bebop[1, :], hist_qd[1, :]])
    all_z = np.concatenate([hist_poi_bebop[2, :], hist_qd[2, :], [0.0]])
    x_min, x_max = float(np.nanmin(all_x)), float(np.nanmax(all_x))
    y_min, y_max = float(np.nanmin(all_y)), float(np.nanmax(all_y))
    z_max = float(np.nanmax(all_z))
    if not np.all(np.isfinite([x_min, x_max, y_min, y_max, z_max])):
        x_min, x_max, y_min, y_max, z_max = -2.0, 2.0, -2.0, 2.0, 1.5
    return (
        x_min - margin,
        x_max + margin,
        y_min - margin,
        y_max + margin,
        max(z_max + margin, 1.5),
    )


def _save_figure(fig: Figure, output_dir: Path, filename: str) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / filename
    fig.savefig(path, dpi=PLOT_DPI, bbox_inches="tight")
    return path


def plot_robot_simulation(
    time: FloatArray,
    hist_limo: FloatArray,
    hist_bebop: FloatArray,
    hist_poi_limo: FloatArray,
    hist_poi_bebop: FloatArray,
    hist_qd: FloatArray,
    poi_offset: float,
    animate: bool = False,
) -> Figure:
    fig = plt.figure(figsize=(12, 5))
    ax3d = fig.add_subplot(121, projection="3d")
    ax2d = fig.add_subplot(122)

    ax3d.plot(
        hist_poi_limo[0, :],
        hist_poi_limo[1, :],
        hist_poi_limo[2, :],
        color="#2563eb",
        alpha=0.35,
        label="LIMO PoI",
    )
    ax3d.plot(
        hist_poi_bebop[0, :],
        hist_poi_bebop[1, :],
        hist_poi_bebop[2, :],
        color="#dc2626",
        alpha=0.35,
        label="Bebop PoI",
    )
    ax3d.plot(
        hist_qd[0, :],
        hist_qd[1, :],
        hist_qd[2, :],
        "k--",
        alpha=0.4,
        label="Desired formation",
    )

    ax2d.plot(hist_limo[0, :], hist_limo[1, :], color="#2563eb", alpha=0.4, label="LIMO")
    ax2d.plot(hist_bebop[0, :], hist_bebop[1, :], color="#dc2626", alpha=0.4, label="Bebop (XY)")
    ax2d.plot(hist_qd[0, :], hist_qd[1, :], "k--", alpha=0.4, label="Desired formation")
    obstacle = plt.Circle(
        OBSTACLE_CENTER,
        OBSTACLE_RADIUS,
        color="#78716c",
        alpha=0.6,
        label="Obstacle",
    )
    influence = plt.Circle(
        OBSTACLE_CENTER,
        OBSTACLE_INFLUENCE,
        color="#78716c",
        fill=False,
        linestyle="--",
        alpha=0.5,
        label="Influence zone",
    )
    ax2d.add_patch(influence)
    ax2d.add_patch(obstacle)
    ax2d.set_xlabel("X (m)")
    ax2d.set_ylabel("Y (m)")
    ax2d.set_title("Top-down view")
    ax2d.axis("equal")
    ax2d.grid(True)
    ax2d.legend(loc="upper right", fontsize=8)

    x_min, x_max, y_min, y_max, z_max = _axis_limits(
        hist_limo, hist_bebop, hist_qd, hist_poi_bebop
    )
    ax3d.set_xlim(x_min, x_max)
    ax3d.set_ylim(y_min, y_max)
    ax3d.set_zlim(0.0, z_max)
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title("LIMO + Bebop 2 simulation")
    ax3d.legend(loc="upper right", fontsize=8)

    xx, yy = np.meshgrid(np.linspace(x_min, x_max, 2), np.linspace(y_min, y_max, 2))
    ax3d.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.08, color="gray")

    if not animate:
        final_step = len(time) - 1
        _draw_limo_3d(ax3d, *hist_limo[:, final_step], poi_offset=poi_offset)
        _draw_bebop_3d(ax3d, *hist_bebop[:3, final_step])
        ax3d.plot(
            [_finite(hist_poi_limo[0, final_step]), _finite(hist_poi_bebop[0, final_step])],
            [_finite(hist_poi_limo[1, final_step]), _finite(hist_poi_bebop[1, final_step])],
            [_finite(hist_poi_limo[2, final_step]), _finite(hist_poi_bebop[2, final_step])],
            color="#16a34a",
            linewidth=2,
            label="Virtual structure",
        )
        ax2d.scatter(
            hist_limo[0, final_step],
            hist_limo[1, final_step],
            color="#2563eb",
            s=60,
            zorder=5,
        )
        ax2d.scatter(
            hist_bebop[0, final_step],
            hist_bebop[1, final_step],
            color="#dc2626",
            s=60,
            zorder=5,
        )
        fig.tight_layout()
        return fig

    limo_body, = ax3d.plot([], [], [], color="#2563eb", linewidth=2)
    limo_poi, = ax3d.plot([], [], [], "o", color="#1d4ed8", markersize=6)
    limo_heading, = ax3d.plot([], [], [], color="black", linewidth=1.5)
    bebop_point, = ax3d.plot([], [], [], "^", color="#dc2626", markersize=10)
    bebop_altitude, = ax3d.plot([], [], [], color="#dc2626", linestyle=":", alpha=0.5)
    virtual_structure, = ax3d.plot([], [], [], color="#16a34a", linewidth=2)
    title = ax3d.set_title("LIMO + Bebop 2 simulation | t = 0.0 s")

    limo_2d, = ax2d.plot([], [], color="#2563eb", linewidth=2)
    bebop_2d, = ax2d.plot([], [], "o", color="#dc2626", markersize=8)

    def update_frame(frame: int) -> tuple[Any, ...]:
        x1, y1, yaw1 = hist_limo[:, frame]
        x2, y2, z2 = hist_bebop[:3, frame]
        px1, py1, pz1 = hist_poi_limo[:, frame]
        px2, py2, pz2 = hist_poi_bebop[:, frame]

        xs, ys, zs = _limo_corners(x1, y1, yaw1)
        _update_line_3d(limo_body, xs, ys, zs)
        _update_point_3d(limo_poi, px1, py1, pz1)

        x1s, y1s, yaw1s = _finite(x1), _finite(y1), _finite(yaw1)
        _update_line_3d(
            limo_heading,
            [x1s, x1s + 0.15 * np.cos(yaw1s)],
            [y1s, y1s + 0.15 * np.sin(yaw1s)],
            [0.0, 0.0],
        )

        x2s, y2s, z2s = _finite(x2), _finite(y2), _finite(z2)
        _update_point_3d(bebop_point, x2s, y2s, z2s)
        _update_line_3d(bebop_altitude, [x2s, x2s], [y2s, y2s], [0.0, z2s])
        _update_line_3d(
            virtual_structure,
            [_finite(px1), _finite(px2)],
            [_finite(py1), _finite(py2)],
            [_finite(pz1), _finite(pz2)],
        )

        xs2d, ys2d, _ = _limo_corners(x1, y1, yaw1)
        limo_2d.set_data(xs2d, ys2d)
        bebop_2d.set_data([x2s], [y2s])

        title.set_text(f"LIMO + Bebop 2 simulation | t = {time[frame]:.1f} s")
        return (
            limo_body,
            limo_poi,
            limo_heading,
            bebop_point,
            bebop_altitude,
            virtual_structure,
            title,
            limo_2d,
            bebop_2d,
        )

    interval_ms = max(int((time[1] - time[0]) * 1000), 20) if len(time) > 1 else 50
    animation = FuncAnimation(
        fig,
        update_frame,
        frames=len(time),
        interval=interval_ms,
        repeat=True,
        blit=False,
    )
    fig._animation = animation
    fig.tight_layout()
    return fig


def plot_results(
    time: FloatArray,
    hist_q: FloatArray,
    hist_qd: FloatArray,
    hist_error: FloatArray,
    hist_limo: FloatArray,
    hist_bebop: FloatArray,
    hist_poi_limo: FloatArray,
    hist_poi_bebop: FloatArray,
    poi_offset: float,
    config: SimulationConfig,
) -> tuple[Figure, Figure]:
    robots_figure = plot_robot_simulation(
        time=time,
        hist_limo=hist_limo,
        hist_bebop=hist_bebop,
        hist_poi_limo=hist_poi_limo,
        hist_poi_bebop=hist_poi_bebop,
        hist_qd=hist_qd,
        poi_offset=poi_offset,
        animate=config.animate,
    )

    errors_figure = plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(time, hist_qd[0, :], "r--", label="Desired Xf")
    plt.plot(time, hist_q[0, :], "b", label="Actual Xf")
    plt.plot(time, hist_qd[1, :], "m--", alpha=0.7, label="Desired Yf")
    plt.plot(time, hist_q[1, :], "c", alpha=0.7, label="Actual Yf")
    plt.grid(True)
    plt.ylabel("Position (m)")
    plt.legend(fontsize=8, ncol=2)
    plt.title("Formation space simulation (Bernoulli lemniscate)")

    plt.subplot(3, 1, 2)
    plt.plot(time, hist_error[0, :], "r", label="Xf error")
    plt.plot(time, hist_error[1, :], "g", label="Yf error")
    plt.plot(time, hist_error[3, :], "b", label="Rho error")
    plt.grid(True)
    plt.ylabel("Position errors (m)")
    plt.legend(fontsize=8)

    plt.subplot(3, 1, 3)
    plt.plot(time, np.rad2deg(hist_error[5, :]), color="#ea580c", label="Beta error (deg)")
    plt.grid(True)
    plt.xlabel("Time (s)")
    plt.ylabel("Beta error (deg)")
    plt.legend(fontsize=8)
    errors_figure.tight_layout()

    robots_path = _save_figure(robots_figure, config.output_dir, ROBOTS_PLOT_NAME)
    errors_path = _save_figure(errors_figure, config.output_dir, ERRORS_PLOT_NAME)
    print(f"Saved robot plot to {robots_path}")
    print(f"Saved error plot to {errors_path}")

    if config.show_plots:
        plt.show()

    return robots_figure, errors_figure
