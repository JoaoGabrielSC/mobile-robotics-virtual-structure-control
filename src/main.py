import shutil
import subprocess
import sys


def ensure_project_env() -> None:
    try:
        import matplotlib  # noqa: F401
        import numpy  # noqa: F401
    except ImportError:
        if shutil.which("uv"):
            subprocess.run(["uv", "run", "python", __file__, *sys.argv[1:]], check=True)
            sys.exit(0)
        print(
            "Dependencies are not installed in this Python environment.\n"
            "Run: uv sync && uv run python src/main.py [args]\n"
            "Or:  source .venv/bin/activate && python src/main.py [args]",
            file=sys.stderr,
        )
        sys.exit(1)


def main() -> None:
    from cli import parse_cli
    from simulator import run_simulation

    config = parse_cli()
    run_simulation(config)


if __name__ == "__main__":
    ensure_project_env()
    main()
