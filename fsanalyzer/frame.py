import numpy as np


class Frame:

    def __init__(self, name="origin", x=0.0, y=0.0, theta=0.0):
        self.name = name
        c = np.cos(np.radians(theta))
        s = np.sin(np.radians(theta))
        self.relative = np.matrix([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])
        self.absolute = np.matrix(np.eye(3, 3))
        self.parent_frame = None
        self.parent_body = None

    def set_parent_frame(self, parent_frame):
        self.parent_frame = parent_frame

    def set_parent_body(self, parent_body):
        self.parent_body = parent_body

    def update(self):
        self.absolute = self.parent_frame.absolute * self.relative

    def draw(self, ax, scale=1.0, draw_label=True):
        origin = self.absolute * np.matrix([[0.0], [0.0], [1.0]])
        x = self.absolute * np.matrix([[scale], [0.0], [1.0]])
        y = self.absolute * np.matrix([[0.0], [scale], [1.0]])
        ax.plot([origin[0, 0], x[0, 0]], [origin[1, 0], x[1, 0]], color="red")
        ax.plot([origin[0, 0], y[0, 0]], [origin[1, 0], y[1, 0]], color="blue")
        if(draw_label):
            ax.text(origin[0, 0], origin[1, 0], self.name)
