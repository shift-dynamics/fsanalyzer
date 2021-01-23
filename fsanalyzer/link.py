import numpy as np

from .frame import Frame


class Link:

    def __init__(self, name):
        self.name = name
        # append origin to list of frames
        self.frames = [Frame()]
        self.points = None

    def add_frame(self, frame):
        frame.set_parent(self.frames[0])
        for f in self.frames:
            if f.name == frame.name:
                raise Exception(
                    f"Cannot add frame {frame.name} to link {self.name}, "
                    "frame with the same name already exists")
        self.frames.append(frame)

    def add_points(self, points):
        self.points = points

    def get_frame(self, name):
        for frame in self.frames:
            if frame.name == name:
                return frame
        else:
            raise Exception(f"Frame {name} not found in body: {self.name}")

    def update(self):
        for frame in self.frames[1:]:
            frame.update()

    def draw(self, ax):
        print(f"Drawing link {self.name}")
        if self.points and len(self.points) > 1:
            origin = self.frames[0].absolute
            x = []
            y = []
            for point in self.points:
                transformed_point = \
                    origin * \
                    np.matrix([[point[0]], [point[1]], [1.0]])
                x.append(transformed_point[0, 0])
                y.append(transformed_point[1, 0])
            ax.plot(x, y, color="green", marker="o")
        else:
            if len(self.frames) > 1:
                offset = [self.frames[0].absolute[0, 2],
                          self.frames[0].absolute[1, 2]]
                for frame in self.frames[1:]:
                    ax.plot([offset[0], frame.absolute[0, 2]],
                            [offset[1], frame.absolute[1, 2]], color="green")
