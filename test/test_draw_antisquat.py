import sys
import os
import argparse
from pathlib import Path
fsanalyzer_path = Path(__file__).parents[1].resolve()
sys.path.append(os.fspath(fsanalyzer_path))

from fsanalyzer.kinematics import FsAnalyzer


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--no-frames', help='disable drawing coordinate frames',
        action='store_true')
    parser.add_argument(
        '--no-labels', help='disable drawing labels',
        action='store_true')
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    file_path = Path(__file__).parents[1] / "data" / "example.yaml"
    fsanalyzer = FsAnalyzer(file_path)
    fsanalyzer.draw(
        draw_frames=not args.no_frames, draw_labels=not args.no_labels)
