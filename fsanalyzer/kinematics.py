#!/usr/bin/env python
import json
import matplotlib.pyplot as plt

from .wheel import Wheel
from .link import Link
from .joint import PrismaticJoint, RevoluteJoint, RigidJoint
from .frame import Frame


class FsAnalyzer:

    def __init__(self, json_file):
        with open(json_file) as f:
            self.config = json.load(f)
        self.links = []
        self.joints = []
        self.generate_kinematics()
        # self.draw()

    def generate_kinematics(self):

        fixed_link = Link("fixed_link")
        self.links.append(fixed_link)

        front_wheel_radius = \
            self.config["wheels"]["front"]["diameter"] / 2.0
        front_wheel = Wheel("front_wheel", front_wheel_radius)
        self.links.append(front_wheel)

        front_axle_mount = RigidJoint(
            fixed_link.get_frame("origin"),
            front_wheel.get_frame("origin"),
            x=0.0, y=front_wheel_radius)
        self.joints.append(front_axle_mount)
        front_axle_mount.update(None, None)

        # add suspension fork link
        front_triangle = None
        for value in self.config["links"]:
            if value["name"] == "front_triangle":
                front_triangle = value
                break
        else:
            raise Exception("No front_triangle link specified in input file")
        suspension_lower = Link("suspension_lower")
        suspension_lower.add_frame(
            Frame(name="distal", x=-self.config["fork"]["offset"],
                  y=self.config["fork"]["axle_to_crown"] / 2.0))
        suspension_distal_frame = suspension_lower.get_frame("distal")
        suspension_lower.add_points(
            [[0.0, 0.0],
             [-self.config["fork"]["offset"], 0.0],
             [-self.config["fork"]["offset"],
              self.config["fork"]["axle_to_crown"] / 2.0]])
        front_axle = RevoluteJoint(
            front_wheel.get_frame("origin"),
            suspension_lower.get_frame("origin"),
            initial_position=90.0 - front_triangle["head_tube_angle"])
        self.joints.append(front_axle)
        suspension_lower.update()
        self.links.append(suspension_lower)

        suspension_upper = Link("suspension_upper")
        suspension_upper.add_frame(
            Frame(name="distal", x=0.0,
                  y=self.config["fork"]["axle_to_crown"] / 2.0))
        suspension = PrismaticJoint(
            suspension_lower.get_frame("distal"),
            suspension_upper.get_frame("origin"),
            initial_position=0.0)
        self.joints.append(suspension)
        suspension_upper.update()

        self.links.append(suspension_upper)

    def draw(self):
        fig, ax = plt.subplots()
        ax.set_aspect(1)
        ax.set_ylim(0.0, 1200.0)
        ax.set_xlim(-2000.0, 500.0)
        for link in self.links:
            link.draw(ax)
            for frame in link.frames:
                frame.draw(ax, scale=50)
        plt.show()
