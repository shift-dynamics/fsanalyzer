import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from fsanalyzer.kinematics import FsAnalyzer

if __name__ == "__main__":
    file_path = os.path.join(os.path.dirname(__file__), "..", "data", "example.yaml")
    fsanalyzer = FsAnalyzer(file_path)
    fsanalyzer.draw()

