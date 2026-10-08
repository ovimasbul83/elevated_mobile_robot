"""Launch the viewer on Windows, Linux, or macOS with the active environment."""
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    runner = Path(__file__).resolve().with_name("run_sim.py")
    executable = sys.executable
    if sys.platform == "darwin":
        local_launcher = Path(sys.executable).parent / "mjpython"
        executable = (
            str(local_launcher) if local_launcher.is_file()
            else shutil.which("mjpython")
        )
        if not executable:
            raise SystemExit(
                "mjpython was not found. Activate the environment and install requirements.txt."
            )
    return subprocess.call([executable, str(runner), *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
