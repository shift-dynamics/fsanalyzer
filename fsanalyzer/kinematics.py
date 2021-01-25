import json
import yaml
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from collections import namedtuple
from math import *  # noqa: F403, F401

from .wheel import Wheel
from .link import Link
from .joint import PrismaticJoint, RevoluteJoint, RigidJoint
from .frame import Frame
from . import geometry


class FsAnalyzer:

    def __init__(self, input_file):
        p = Path(input_file)
        if not p.exists():
            raise Exception(f"Input file {input_file} does not exist")
        if p.suffix == ".yaml":
            print(f"loading yaml file: {input_file}")
            with p.open() as f:
                self.config = yaml.safe_load(f)
        elif p.suffix == ".json":
            print(f"loading json file: {input_file}")
            with p.open() as f:
                self.config = json.load(f)
        else:
            raise Exception(f"Unrecognized extension '{p.suffix}', only json "
                            "and yaml supported")
        self.links = []
        self.joints = []
        self.generate_kinematics()

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
            initial_position=90.0 - self.config["geometry"]["head_tube_angle"])
        self.joints.append(front_axle)
        suspension_lower.update()
        self.links.append(suspension_lower)

        suspension_upper = Link("suspension_upper")
        suspension_upper.add_frame(
            Frame(name="distal", x=0.0,
                  y=self.config["fork"]["axle_to_crown"] / 2.0,
                  theta=self.config["geometry"]["head_tube_angle"] - 90.0))
        suspension = PrismaticJoint(
            suspension_lower.get_frame("distal"),
            suspension_upper.get_frame("origin"),
            name="front_suspension",
            initial_position=0.0)
        self.joints.append(suspension)
        suspension_upper.update()

        self.links.append(suspension_upper)

        # draw frame front triangle
        # get position of bottom of steerer tube
        lower_steerer_tube = \
            suspension_upper.get_frame("distal").get_position()

        # height of bottom bracket
        bb_height_absolute = \
            front_wheel_radius - self.config["geometry"]["bb_drop"]

        front_triangle = Link("front_triangle")

        top_of_head_tube_absolute = \
            bb_height_absolute + self.config["geometry"]["stack"]
        y = top_of_head_tube_absolute - lower_steerer_tube[1]
        x = -y * np.tan(
            np.radians(90.0 - self.config["geometry"]["head_tube_angle"]))

        front_triangle.add_frame(
            Frame(name="top_of_head_tube", x=x, y=y))
        front_triangle.add_frame(
            Frame(name="bottom_bracket",
                  x=-self.config["geometry"]["reach"] + x,
                  y=-lower_steerer_tube[1] + bb_height_absolute))
        head_tube_joint = RigidJoint(
            suspension_upper.get_frame("distal"),
            front_triangle.get_frame("origin"))
        head_tube_joint.update(None, None)
        self.joints.append(head_tube_joint)

        # add links specified in input file
        front_triangle_config = None
        for link in self.config["links"]:
            if link["name"] == "front_triangle":
                front_triangle_config = link
                break
        else:
            raise Exception(
                "Could not find link `front_triangle` in input file")
        Vec2 = namedtuple('Vec2', 'x, y')
        bottom_bracket = Vec2(
            front_triangle.get_frame("bottom_bracket").relative[0, 2],
            front_triangle.get_frame("bottom_bracket").relative[1, 2])

        # list of safe methods for use in code
        safe_list = ['acos', 'asin', 'atan', 'atan2', 'ceil', 'cos',
                     'cosh', 'degrees', 'e', 'exp', 'fabs', 'floor',
                     'fmod', 'frexp', 'hypot', 'ldexp', 'log', 'log10',
                     'modf', 'pi', 'pow', 'radians', 'sin', 'sinh', 'sqrt',
                     'tan', 'tanh']
        safe_dict = dict([(k, locals().get(k, None)) for k in safe_list])
        safe_dict["bottom_bracket"] = bottom_bracket

        for frame in front_triangle_config["frames"]:
            x_offset = None
            y_offset = None
            try:
                x_offset = float(frame["offset"][0])
            except ValueError:
                try:
                    x_offset = float(eval(frame["offset"][0],
                                     {"__builtins__": None}, safe_dict))
                except ValueError:
                    raise Exception(
                        f"Problem parsing x offset in {frame['name']}")
            try:
                y_offset = float(frame["offset"][0])
            except ValueError:
                try:
                    y_offset = float(eval(frame["offset"][1],
                                     {"__builtins__": None}, safe_dict))
                except ValueError:
                    raise Exception(
                        f"Problem parsing y offset in {frame['name']}")
            front_triangle.add_frame(
                Frame(name=frame["name"], x=x_offset, y=y_offset))

        front_triangle.update()
        self.links.append(front_triangle)

        # add rear linkage by adding remaining links in input file
        for link_ in self.config["links"]:
            if link_["name"] == "front_triangle":
                # we already added the front_triangle above
                continue
            link = Link(link_["name"])
            for frame in link_["frames"]:
                link.add_frame(
                    Frame(name=frame["name"],
                          x=frame["offset"][0],
                          y=frame["offset"][1]))
            self.links.append(link)

        for joint_ in self.config["joints"]:
            parent = \
                self.get_link(joint_["link_frame_pair"][0][0]).get_frame(
                    joint_["link_frame_pair"][0][1])
            child = \
                self.get_link(joint_["link_frame_pair"][1][0]).get_frame(
                    joint_["link_frame_pair"][1][1])
            joint = RevoluteJoint(parent, child, name=joint_["name"])
            joint.child.parent_body.update()
            self.joints.append(joint)

        seat_stay_distal_abs = \
            self.get_link("rocker_link").get_frame(
                "seat_stay_attachment").get_position()
        seat_stay_link = self.get_link("seat_stay_link")
        seat_stay_proximal_abs = seat_stay_link.get_frame(
            "origin").get_position()
        relative_pos = \
            seat_stay_distal_abs - seat_stay_proximal_abs
        rocker_link_attachment_frame = seat_stay_link.get_frame(
            "rocker_link_attachment")
        rocker_link_attachment_frame.relative[0, 2] = relative_pos[0]
        rocker_link_attachment_frame.relative[1, 2] = relative_pos[1]
        rocker_link_attachment_frame.update()

        # add rear shock
        shock_lower_pos = self.get_link("front_triangle").get_frame(
            "shock_attachment").get_position()
        shock_upper_pos = self.get_link("rocker_link").get_frame(
            "shock_attachment").get_position()
        print(
            "shock distance: ",
            np.linalg.norm(shock_lower_pos - shock_upper_pos))

        shock_lower = Link("shock_lower")
        shock_lower.add_frame(
            Frame(name="distal",
                  x=0.0,
                  y=self.config["shock"]["eye_to_eye"] / 2.0))

        shock_upper = Link("shock_upper")
        shock_upper.add_frame(
            Frame(name="distal",
                  x=0.0,
                  y=self.config["shock"]["eye_to_eye"] / 2.0))

        # add chainrings
        for i, ring in \
                enumerate(self.config["drive_train"]["front_chainrings"]):
            sprocket = Wheel(
                f"front_sprocket_{i}", geometry.chain_ring_radius(ring))
            joint = RigidJoint(front_triangle.get_frame("bottom_bracket"),
                               sprocket.get_frame("origin"))
            self.joints.append(joint)
            joint.update(None, None)
            self.links.append(sprocket)

        # add a placeholder rear wheel
        r = self.config["geometry"]["chain_stay_length"]
        x = np.sqrt(np.abs(r**2 - self.config["geometry"]["bb_drop"]**2))
        fixed_link.add_frame(
            Frame(name="rear_wheel",
                  x=-x + front_triangle.get_frame(
                      "bottom_bracket").get_position()[0],
                  y=front_wheel_radius))
        fixed_link.update()
        print(front_triangle.get_frame("bottom_bracket").get_position()[0])

        rear_wheel = Wheel("rear_wheel", front_wheel_radius)
        rear_wheel_attachment = RigidJoint(
            fixed_link.get_frame("rear_wheel"),
            rear_wheel.get_frame("origin"))
        rear_wheel_attachment.update(None, None)
        self.links.append(rear_wheel)

        for i, ring in \
                enumerate(self.config["drive_train"]["rear_chainrings"]):
            r2 = geometry.chain_ring_radius(ring)
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

    def get_joint(self, name):
        for joint in self.joints:
            if joint.name == name:
                return joint
        else:
            raise Exception(f"Could not find joint {name}")

    def draw_chainlines(self, ax, ax2=None):
        front_triangle = self.get_link("front_triangle")
        p1 = self.get_link("front_triangle").get_frame(
            "bottom_bracket").get_position()
        p2 = self.get_link("rear_wheel").get_frame(
            "origin").get_position()

        r1 = geometry.chain_ring_radius(
            self.config["drive_train"]["front_chainrings"][0])

        ic = None
        attachment_link = self.config["wheels"]["rear"]["attachment_link"]
        if attachment_link == "seat_stay_link":
            l1 = [self.get_link("chain_stay_link").get_frame(
                  "seat_stay_attachment").get_position(),
                  self.get_link("chain_stay_link").get_frame(
                  "origin").get_position()]
            l2 = [self.get_link("rocker_link").get_frame(
                  "seat_stay_attachment").get_position(),
                  self.get_link("rocker_link").get_frame(
                  "origin").get_position()]
            ic = geometry.line_intersection(l1, l2)
        elif attachment_link == "chain_stay_link":
            ic = self.get_link("front_triangle").get_frame(
                "chain_stay_attachment").get_position()
        else:
            raise Exception(
                "invalid setting found for wheel `attachment_link`, "
                f"{attachment_link} was specified, but only `chain_stay_link`"
                "and `seat_stay_link` are valid values")
        # draw instant center (@TODO get this from suspension geometry, just
        # adding a placeholder for now)
        ax.plot([ic[0], ic[0] + 50.0], [ic[1], ic[1]], color="red")
        ax.plot([ic[0], ic[0]], [ic[1], ic[1] + 50], color="blue")

        ax.plot([p2[0], ic[0]], [p2[1], ic[1]], color="black",
                linestyle="--")
        ax.text(ic[0], ic[1], "instant_center")

        d1 = p2 - p1
        d1_norm = np.linalg.norm(d1)
        theta1 = np.pi - np.arctan2(d1[1], d1[0])

        com = self.config["center_of_mass"]["height"]
        antisquat = []
        for i, ring in \
                enumerate(self.config["drive_train"]["rear_chainrings"]):
            r2 = geometry.chain_ring_radius(ring)
            theta2 = np.arccos((r2 - r1) / d1_norm)
            theta3 = theta1 - theta2

            c1 = [p1[0] + r1 * np.cos(theta3),
                  p1[1] - r1 * np.sin(theta3)]

            c2 = [p2[0] + r2 * np.cos(theta3),
                  p2[1] - r2 * np.sin(theta3)]

            ax.plot([c1[0], c2[0]], [c1[1], c2[1]])

            # find intersection between chainline and line drawn from rear
            # hub to instant center
            intercept = geometry.line_intersection(
                [c1, c2], [[p2[0], p2[1]], ic])
            ax.plot(intercept[0], intercept[1], color="red", marker="o")

            antisquat_intercept = geometry.line_intersection(
                [[p2[0], 0.0], intercept], [[0.0, 0.0], [0.0, 100.0]])
            ax.plot([p2[0], antisquat_intercept[0]],
                    [0.0, antisquat_intercept[1]], color="red")
            antisquat_pct = antisquat_intercept[1] / com * 100.0
            antisquat.append(antisquat_pct)

        if ax2 is not None:
            ax2.scatter([0] * len(antisquat), antisquat)

        bb_x = front_triangle.get_frame("bottom_bracket").get_position()[0]
        ax.plot([0.0, 0.0], [0.0, com + 50.0],
                color="black", linestyle="--")
        ax.plot([bb_x, 50.0], [com, com], color="black", linestyle="--")
        ax.text(50.0, com, "100% antisquat", verticalalignment="center")

    def draw(self, draw_frames=True, draw_labels=True):
        fig, ax = plt.subplots(2, 1)
        ax[0].set_aspect(1)

        upper_lim = \
            ceil(self.config["center_of_mass"]["height"] /  # noqa
                 100) * 100 + 100
        ax[0].set_ylim(0.0, upper_lim)

        wheel_radius = self.config["wheels"]["front"]["diameter"] / 2.0
        right_lim = \
            ceil((self.get_link("front_wheel").get_frame(  # noqa
                 "origin").get_position()[0] + wheel_radius) / 100) * 100 + 100
        left_lim = \
            floor((self.get_link("rear_wheel").get_frame(  # noqa
                  "origin").get_position()[0] - wheel_radius) / 100) * \
            100 - 100
        ax[0].set_xlim(left_lim, right_lim)

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
            if draw_frames:
                for frame in link.frames:
                    frame.draw(ax[0], scale=50, draw_labels=draw_labels)
        self.draw_chainlines(ax[0], ax[1])
        plt.show()
