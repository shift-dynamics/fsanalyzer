#!/usr/bin/env python
import json
import matplotlib.pyplot as plt
import numpy as np

from .wheel import Wheel
from .link import Link
from .joint import PrismaticJoint, RevoluteJoint, RigidJoint
from .frame import Frame


def chain_ring_radius(n, pitch=12.7):
    return pitch / (2 * np.sin(np.pi / n))


def line_intersection(l1, l2):
    p1 = l1[0]
    p2 = l1[1]
    p3 = l2[0]
    p4 = l2[1]
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x3 = p3[0]
    y3 = p3[1]
    x4 = p4[0]
    y4 = p4[1]
    return [
        ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3  * x4)) /
        ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)),
        ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3  * x4)) /
        ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
    ]


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
                  y=self.config["fork"]["axle_to_crown"] / 2.0,
                  theta=front_triangle["head_tube_angle"] - 90.0))
        suspension = PrismaticJoint(
            suspension_lower.get_frame("distal"),
            suspension_upper.get_frame("origin"),
            initial_position=0.0)
        self.joints.append(suspension)
        suspension_upper.update()

        self.links.append(suspension_upper)

        # draw frame
        # get position of bottom of steerer tube
        lower_steerer_tube = suspension_upper.get_frame("distal").absolute[:, 2][:2]
        print(lower_steerer_tube)

        # height of bottom bracket
        bb_height_absolute = front_wheel_radius - front_triangle["bb_drop"]
        frame = Link("frame")

        top_of_head_tube_absolute = bb_height_absolute + front_triangle["stack"]
        y = top_of_head_tube_absolute - lower_steerer_tube[1, 0]
        x = -y * np.tan(np.radians(90.0 - front_triangle["head_tube_angle"]))

        frame.add_frame(
            Frame(name="top_of_head_tube", x=x, y=y))
        frame.add_frame(
            Frame(name="bottom_bracket",
                  x=-front_triangle["reach"] + x,
                  y=-lower_steerer_tube[1, 0] + bb_height_absolute))
        head_tube_joint = RigidJoint(
            suspension_upper.get_frame("distal"),
            frame.get_frame("origin"))
        head_tube_joint.update(None, None)
        self.joints.append(head_tube_joint)
        frame.update()

        frame.update()
        self.links.append(frame)

        for i, ring in \
                enumerate(self.config["drive_train"]["front_chainrings"]):
            sprocket = Wheel(f"front_sprocket_{i}", chain_ring_radius(ring))
            joint = RigidJoint(frame.get_frame("bottom_bracket"),
                               sprocket.get_frame("origin"))
            self.joints.append(joint)
            joint.update(None, None)
            self.links.append(sprocket)

        # add a placeholder rear wheel
        r = self.config["wheels"]["rear"]["chain_stay_length"]
        x = np.sqrt(np.abs(r**2 - front_triangle["bb_drop"]**2))
        fixed_link.add_frame(
            Frame(name="rear_wheel",
                  x=-x + frame.get_frame("bottom_bracket").absolute[0, 2],
                  y=front_wheel_radius))
        fixed_link.update()
        print(frame.get_frame("bottom_bracket").absolute[0, 2])

        rear_wheel = Wheel("rear_wheel", front_wheel_radius)
        rear_wheel_attachment = RigidJoint(
            fixed_link.get_frame("rear_wheel"),
            rear_wheel.get_frame("origin"))
        rear_wheel_attachment.update(None, None)
        self.links.append(rear_wheel)

        for i, ring in \
                enumerate(self.config["drive_train"]["rear_chainrings"]):
            r2 = chain_ring_radius(ring)
            sprocket = Wheel(f"rear_sprocket_{i}", r2)
            joint = RigidJoint(rear_wheel.get_frame("origin"),
                               sprocket.get_frame("origin"))
            self.joints.append(joint)
            joint.update(None, None)
            self.links.append(sprocket)

    def get_link(self, name):
        for link in self.links:
            if link.name == name:
                return link
        else:
            raise Exception(f"Could not find link {name}")

    def draw_chainlines(self, ax, ax2=None):
        frame = self.get_link("frame")
        rear_wheel = self.get_link("rear_wheel")

        r1 = chain_ring_radius(
            self.config["drive_train"]["front_chainrings"][0])
        p1 = frame.get_frame("bottom_bracket").absolute[:2, 2]
        p2 = rear_wheel.get_frame("origin").absolute[:2, 2]

        # draw instant center (@TODO get this from suspension geometry, just
        # adding a placeholder for now)
        ic = [p1[0, 0] + 120.0, p1[1, 0] + 73.0]
        ax.plot([ic[0], ic[0] + 50.0], [ic[1], ic[1]], color="red")
        ax.plot([ic[0], ic[0]], [ic[1], ic[1] + 50], color="blue")
        ax.plot([p2[0, 0], ic[0]], [p2[1, 0], ic[1]], color="black",
                linestyle="--")
        ax.text(ic[0], ic[1], "instant_center")

        d1 = p2 - p1
        d1_norm = np.linalg.norm(d1)
        theta1 = np.pi - np.arctan2(d1[1, 0], d1[0, 0])

        com = self.config["center_of_mass"]["height"]
        antisquat = []
        for i, ring in \
                enumerate(self.config["drive_train"]["rear_chainrings"]):
            r2 = chain_ring_radius(ring)
            theta2 = np.arccos((r2 - r1) / d1_norm)
            theta3 = theta1 - theta2

            c1 = [p1[0, 0] + r1 * np.cos(theta3),
                  p1[1, 0] - r1 * np.sin(theta3)]

            c2 = [p2[0, 0] + r2 * np.cos(theta3),
                  p2[1, 0] - r2 * np.sin(theta3)]

            ax.plot([c1[0], c2[0]], [c1[1], c2[1]])

            # find intersection between chainline and line drawn from rear
            # hub to instant center
            intercept = line_intersection([c1, c2], [[p2[0, 0], p2[1, 0]], ic])
            ax.plot(intercept[0], intercept[1], color="red", marker="o")

            antisquat_intercept = line_intersection(
                [[p2[0, 0], 0.0], intercept], [[0.0, 0.0], [0.0, 100.0]])
            ax.plot([p2[0, 0], antisquat_intercept[0]],
                    [0.0, antisquat_intercept[1]], color="red")
            antisquat_pct = antisquat_intercept[1] / com * 100.0
            antisquat.append(antisquat_pct)

        if ax2 is not None:
            ax2.scatter([0] * len(antisquat), antisquat)

        bb_x = frame.get_frame("bottom_bracket").absolute[0, 2]
        ax.plot([0.0, 0.0], [0.0, com + 50.0],
                color="black", linestyle="--")
        ax.plot([bb_x, 50.0], [com, com], color="black", linestyle="--")
        ax.text(50.0, com, "100% antisquat", verticalalignment="center")






    def draw(self):
        fig, ax = plt.subplots(2, 1)
        ax[0].set_aspect(1)
        ax[0].set_ylim(0.0, 1200.0)
        ax[0].set_xlim(-2000.0, 500.0)

        travel = 120.0
        sag = 0.3
        ax[1].set_ylim(-25, 150)
        ax[1].set_ylabel("antisquat (%)")
        ax[1].yaxis.grid(color='gray', linestyle='dashed')
        ax[1].set_xlim(0, travel)
        ax[1].set_xlabel("Rear wheel travel (mm)")

        sag_line = travel * sag
        ax[1].plot([sag_line, sag_line], [-25, 150],
                   color="gray", linestyle="dashed")

        for link in self.links:
            if link.name != "fixed_link":
                link.draw(ax[0])
            for frame in link.frames:
                frame.draw(ax[0], scale=50)
        self.draw_chainlines(ax[0], ax[1])
        plt.show()
