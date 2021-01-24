import sys
import os
from pathlib import Path
fsanalyzer_path = Path(__file__).parents[1].resolve()
sys.path.append(os.fspath(fsanalyzer_path))

from fsanalyzer.kinematics import FsAnalyzer


if __name__ == "__main__":
    file_path = Path(__file__).parents[1] / "data" / "example.yaml"
    fsanalyzer = FsAnalyzer(file_path)
    fsanalyzer.draw()
