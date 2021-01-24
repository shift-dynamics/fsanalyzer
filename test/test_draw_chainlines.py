#!/usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


def chain_ring_radius(n, pitch=12.7):
    return pitch / (2 * np.sin(np.pi / n))


if __name__ == "__main__":

    fig, ax = plt.subplots()
    ax.set_aspect(1)
    ax.set_ylim(-180, 180)
    ax.set_xlim(-550, 100)
    front_chainring = 32
    rear_chainrings = [10, 12, 14, 16, 18, 21, 24, 28, 32, 36, 42, 50]
    p1 = np.array([0.0, 0.0])
    p2 = np.array([-435.0, 50.0])

    r1 = chain_ring_radius(front_chainring)
    d1 = p2 - p1
    theta1 = np.pi - np.arctan2(d1[1], d1[0])

    ax.add_artist(Circle(p1, r1, fill=False, edgecolor="blue"))

    for ring in rear_chainrings:
        r2 = chain_ring_radius(ring)
        ax.add_artist(Circle(p2, r2, fill=False, edgecolor="blue"))
        theta2 = np.arccos((r2 - r1) / np.linalg.norm(d1))
        theta3 = theta1 - theta2

        c1 = [p1[0] + r1 * np.cos(theta3),
              p1[1] - r1 * np.sin(theta3)]

        c2 = [p2[0] + r2 * np.cos(theta3),
              p2[1] - r2 * np.sin(theta3)]

        ax.plot([c1[0], c2[0]], [c1[1], c2[1]])

    plt.show()
