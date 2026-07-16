#!/usr/bin/env python3
"""Gera gráficos e uma animação a partir de um audit_formacao_*.txt."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle


FLOAT_LINE = re.compile(r"^\s*([+-]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:e[+-]?\d+)?)\s*$", re.I)
TIME_LINE = re.compile(r"t\s*=\s*([+-]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:e[+-]?\d+)?)\s*s", re.I)
SCALAR_LINE = r"([+-]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:e[+-]?\d+)?)"


@dataclass
class AuditData:
    time: np.ndarray
    poi: np.ndarray
    reference: np.ndarray
    bebop: np.ndarray
    target: np.ndarray
    command: np.ndarray
    kinematic_residual: np.ndarray
    dynamic_residual: np.ndarray
    saturation_error: np.ndarray
    metadata: dict[str, str]
    execution_summary: dict[str, float]


def vector_after_label(block: str, label: str, size: int) -> np.ndarray | None:
    """Extrai as primeiras `size` linhas numéricas após um rótulo literal."""
    label_index = block.find(label)
    if label_index < 0:
        return None

    values: list[float] = []
    for line in block[label_index + len(label) :].splitlines()[1:]:
        match = FLOAT_LINE.match(line)
        if match:
            values.append(float(match.group(1)))
            if len(values) == size:
                return np.array(values, dtype=float)
        elif values:
            break
    return None


def scalar_after_label(block: str, label: str) -> float:
    match = re.search(re.escape(label) + r"\s*" + SCALAR_LINE, block, flags=re.I)
    return float(match.group(1)) if match else np.nan


def parse_metadata(text: str) -> dict[str, str]:
    metadata: dict[str, str] = {}
    for line in text.splitlines():
        if line.startswith("================================================================"):
            break
        if ":" in line:
            key, value = line.split(":", maxsplit=1)
            metadata[key.strip()] = value.strip()
    return metadata


def parse_audit(path: Path) -> AuditData:
    text = path.read_text(encoding="utf-8", errors="replace")
    metadata = parse_metadata(text)
    blocks = re.split(r"={20,}\s*\n", text)

    rows: list[dict[str, np.ndarray | float]] = []
    for block in blocks:
        time_match = TIME_LINE.search(block)
        if not time_match:
            continue

        poi = vector_after_label(block, "PoI LIMO [m]:", 2)
        reference = vector_after_label(block, "Referência lemniscata [m]:", 2)
        bebop = vector_after_label(block, "p2 medido ou virtual [m]:", 3)
        target = vector_after_label(block, "p2d de formação [m]:", 3)
        command = vector_after_label(block, "Comando após saturação:", 4)
        if any(value is None for value in (poi, reference, bebop, target, command)):
            continue

        rows.append(
            {
                "time": float(time_match.group(1)),
                "poi": poi,
                "reference": reference,
                "bebop": bebop,
                "target": target,
                "command": command,
                "kinematic_residual": scalar_after_label(
                    block, "Norma do resíduo cinemático:"
                ),
                "dynamic_residual": scalar_after_label(
                    block, "Norma do resíduo dinâmico bruto:"
                ),
                "saturation_error": vector_after_label(
                    block, "Erro causado pela saturação:", 4
                ),
            }
        )

    if len(rows) < 2:
        raise ValueError(
            "Não foram encontradas ao menos duas amostras completas. "
            "Use um audit_formacao no formato atual."
        )

    def matrix(name: str) -> np.ndarray:
        values = [row[name] for row in rows]
        return np.asarray(values, dtype=float)

    saturation = matrix("saturation_error")
    summary_labels = {
        "samples": "Amostras:",
        "rms_error": "Erro RMS do Bebop [m]:",
        "max_error": "Erro máximo do Bebop [m]:",
        "final_error": "Erro final do Bebop [m]:",
        "min_obstacle_distance": "Distância mínima LIMO-obstáculo [m]:",
    }
    execution_summary = {
        name: scalar_after_label(text, label) for name, label in summary_labels.items()
    }
    saturation_match = re.search(
        r"Amostras com saturação do Bebop:\s*(\d+)\s+de\s+(\d+)", text, flags=re.I
    )
    if saturation_match:
        execution_summary["saturated_samples"] = float(saturation_match.group(1))
        execution_summary["summary_samples"] = float(saturation_match.group(2))

    return AuditData(
        time=np.asarray([row["time"] for row in rows], dtype=float),
        poi=matrix("poi"),
        reference=matrix("reference"),
        bebop=matrix("bebop"),
        target=matrix("target"),
        command=matrix("command"),
        kinematic_residual=np.asarray(
            [row["kinematic_residual"] for row in rows], dtype=float
        ),
        dynamic_residual=np.asarray(
            [row["dynamic_residual"] for row in rows], dtype=float
        ),
        saturation_error=np.linalg.norm(saturation, axis=1),
        metadata=metadata,
        execution_summary=execution_summary,
    )


def draw_obstacle(ax: plt.Axes, center: np.ndarray, radius: float, influence: float) -> None:
    ax.add_patch(Circle(center, radius, color="tab:red", alpha=0.25, label="Obstáculo"))
    ax.add_patch(
        Circle(
            center,
            influence,
            fill=False,
            color="tab:red",
            linestyle=":",
            label="Zona de influência",
        )
    )


def save_trajectory(
    data: AuditData, output_dir: Path, obstacle: np.ndarray, radius: float, influence: float
) -> None:
    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    draw_obstacle(ax, obstacle, radius, influence)
    ax.plot(data.reference[:, 0], data.reference[:, 1], "k--", label="Referência")
    ax.plot(data.poi[:, 0], data.poi[:, 1], color="tab:blue", label="PoI LIMO")
    ax.plot(data.bebop[:, 0], data.bebop[:, 1], color="tab:orange", label="Bebop")
    ax.plot(data.target[:, 0], data.target[:, 1], color="tab:green", label="Alvo Bebop")
    ax.scatter(*data.poi[0], color="tab:blue", marker="o", zorder=4)
    ax.scatter(*data.bebop[0, :2], color="tab:orange", marker="o", zorder=4)
    ax.set(title="Trajetória reconstruída da auditoria", xlabel="x [m]", ylabel="y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="best")
    fig.savefig(output_dir / "trajetoria_xy.png", dpi=160)
    plt.close(fig)


def save_signals(data: AuditData, output_dir: Path, obstacle: np.ndarray) -> None:
    error = data.target - data.bebop
    error_norm = np.linalg.norm(error, axis=1)
    obstacle_distance = np.linalg.norm(data.poi - obstacle, axis=1)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True, constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(data.time, error[:, 0], label="e_x")
    ax.plot(data.time, error[:, 1], label="e_y")
    ax.plot(data.time, error[:, 2], label="e_z")
    ax.plot(data.time, error_norm, "k", linewidth=1.5, label="||e||")
    ax.set(title="Erro de formação", ylabel="erro [m]")
    ax.grid(True)
    ax.legend()

    ax = axes[0, 1]
    labels = ("u_x", "u_y", "u_z", "u_psi")
    for column, label in enumerate(labels):
        ax.plot(data.time, data.command[:, column], label=label)
    ax.set(title="Comando Bebop após saturação", ylabel="comando")
    ax.grid(True)
    ax.legend()

    ax = axes[1, 0]
    ax.plot(data.time, obstacle_distance, color="tab:purple", label="PoI ao centro")
    ax.set(title="Distância do PoI ao obstáculo", xlabel="tempo [s]", ylabel="distância [m]")
    ax.grid(True)
    ax.legend()

    ax = axes[1, 1]
    ax.semilogy(data.time, np.maximum(data.kinematic_residual, 1e-20), label="cinemático")
    ax.semilogy(data.time, np.maximum(data.dynamic_residual, 1e-20), label="dinâmico")
    ax.semilogy(
        data.time,
        np.maximum(data.saturation_error, 1e-20),
        label="erro de saturação",
    )
    ax.set(title="Resíduos e saturação", xlabel="tempo [s]", ylabel="norma")
    ax.grid(True, which="both")
    ax.legend()

    fig.savefig(output_dir / "sinais_e_metricas.png", dpi=160)
    plt.close(fig)


def save_animation(
    data: AuditData,
    output_dir: Path,
    obstacle: np.ndarray,
    radius: float,
    influence: float,
    fps: int,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    draw_obstacle(ax, obstacle, radius, influence)
    obstacle_extent = np.array(
        [
            obstacle + [-influence, -influence],
            obstacle + [influence, influence],
        ]
    )
    coordinates = np.vstack(
        (data.reference, data.poi, data.bebop[:, :2], data.target[:, :2], obstacle_extent)
    )
    margin = 0.15
    x_center = (coordinates[:, 0].min() + coordinates[:, 0].max()) / 2
    y_center = (coordinates[:, 1].min() + coordinates[:, 1].max()) / 2
    half_span = max(np.ptp(coordinates[:, 0]), np.ptp(coordinates[:, 1])) / 2 + margin
    ax.set_xlim(x_center - half_span, x_center + half_span)
    ax.set_ylim(y_center - half_span, y_center + half_span)
    ax.set(xlabel="x [m]", ylabel="y [m]", title="Reprodução da auditoria")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax.plot(
        data.reference[:, 0],
        data.reference[:, 1],
        "k--",
        alpha=0.35,
        label="Lemniscata completa",
    )
    limo_line, = ax.plot([], [], color="tab:blue", label="PoI LIMO")
    bebop_line, = ax.plot([], [], color="tab:orange", label="Bebop")
    target_line, = ax.plot([], [], color="tab:green", label="Alvo Bebop")
    current_reference, = ax.plot([], [], "ko")
    current_limo, = ax.plot([], [], "o", color="tab:blue")
    current_bebop, = ax.plot([], [], "o", color="tab:orange")
    current_target, = ax.plot([], [], "o", color="tab:green")
    title = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")
    ax.legend(loc="best")

    def update(frame: int):
        limo_line.set_data(data.poi[: frame + 1, 0], data.poi[: frame + 1, 1])
        bebop_line.set_data(data.bebop[: frame + 1, 0], data.bebop[: frame + 1, 1])
        target_line.set_data(data.target[: frame + 1, 0], data.target[: frame + 1, 1])
        current_reference.set_data(
            [data.reference[frame, 0]], [data.reference[frame, 1]]
        )
        current_limo.set_data([data.poi[frame, 0]], [data.poi[frame, 1]])
        current_bebop.set_data([data.bebop[frame, 0]], [data.bebop[frame, 1]])
        current_target.set_data([data.target[frame, 0]], [data.target[frame, 1]])
        title.set_text(f"t = {data.time[frame]:.1f} s")
        return (
            limo_line,
            bebop_line,
            target_line,
            current_reference,
            current_limo,
            current_bebop,
            current_target,
            title,
        )

    animation = FuncAnimation(fig, update, frames=len(data.time), interval=1000 / fps, blit=True)
    animation.save(output_dir / "reproducao.gif", writer=PillowWriter(fps=fps))
    plt.close(fig)


def print_summary(data: AuditData, obstacle: np.ndarray) -> None:
    error_norm = np.linalg.norm(data.target - data.bebop, axis=1)
    obstacle_distance = np.linalg.norm(data.poi - obstacle, axis=1)
    saturation_samples = int(np.count_nonzero(data.saturation_error > 1e-9))
    print("Métricas das amostras registradas no TXT:")
    print(f"  Amostras interpretadas: {len(data.time)}")
    print(f"  Intervalo: {data.time[0]:.3f} s a {data.time[-1]:.3f} s")
    print(f"  Erro RMS do Bebop: {np.sqrt(np.mean(error_norm**2)):.6f} m")
    print(f"  Erro máximo do Bebop: {error_norm.max():.6f} m")
    print(f"  Erro final do Bebop: {error_norm[-1]:.6f} m")
    print(f"  Distância mínima PoI-obstáculo: {obstacle_distance.min():.6f} m")
    print(f"  Amostras com saturação: {saturation_samples} de {len(data.time)}")
    if not np.isnan(data.execution_summary["rms_error"]):
        print("Resumo integral calculado pelo MATLAB:")
        print(f"  Amostras: {data.execution_summary['samples']:.0f}")
        print(f"  Erro RMS do Bebop: {data.execution_summary['rms_error']:.6f} m")
        print(f"  Erro máximo do Bebop: {data.execution_summary['max_error']:.6f} m")
        print(f"  Erro final do Bebop: {data.execution_summary['final_error']:.6f} m")
        print(
            "  Distância mínima LIMO-obstáculo: "
            f"{data.execution_summary['min_obstacle_distance']:.6f} m"
        )
        if "saturated_samples" in data.execution_summary:
            print(
                "  Amostras com saturação: "
                f"{data.execution_summary['saturated_samples']:.0f} de "
                f"{data.execution_summary['summary_samples']:.0f}"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audit", type=Path, help="caminho para audit_formacao_*.txt")
    parser.add_argument("--output-dir", type=Path, help="diretório dos arquivos gerados")
    parser.add_argument("--gif", action="store_true", help="gera reproducao.gif")
    parser.add_argument("--fps", type=int, default=10, help="quadros por segundo do GIF")
    parser.add_argument(
        "--obstacle-center", nargs=2, type=float, default=(-0.20, 0.425), metavar=("X", "Y")
    )
    parser.add_argument("--obstacle-radius", type=float, default=0.15)
    parser.add_argument("--influence-radius", type=float, default=0.25)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.audit.is_file():
        print(f"Arquivo não encontrado: {args.audit}", file=sys.stderr)
        return 2

    output_dir = args.output_dir or args.audit.parent / f"{args.audit.stem}_visualizacao"
    output_dir.mkdir(parents=True, exist_ok=True)
    obstacle = np.asarray(args.obstacle_center, dtype=float)

    try:
        data = parse_audit(args.audit)
        print_summary(data, obstacle)
        save_trajectory(data, output_dir, obstacle, args.obstacle_radius, args.influence_radius)
        save_signals(data, output_dir, obstacle)
        if args.gif:
            save_animation(
                data,
                output_dir,
                obstacle,
                args.obstacle_radius,
                args.influence_radius,
                args.fps,
            )
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Falha ao processar a auditoria: {error}", file=sys.stderr)
        return 1

    print(f"Arquivos salvos em: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
