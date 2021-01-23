import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from fsanalyzer.kinematics import FsAnalyzer

if __name__ == "__main__":
    json_path = os.path.join(os.path.dirname(__file__), "..", "data", "example.json")
    analyzer = FsAnalyzer(json_path)
    analyzer.draw()

