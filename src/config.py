from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SimulationConfig:
    t_final: float
    dt: float
    kq: float
    lq: float
    kd_limo: float
    animate: bool
    output_dir: Path
    show_plots: bool
