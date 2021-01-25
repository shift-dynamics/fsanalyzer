from .link import Link
from .frame import Frame
from matplotlib.patches import Circle


class Wheel(Link):

    def __init__(self, name, radius, color='Blue'):
        self.name = name
        origin = Frame()
        origin.set_parent_body(self)
        self.frames = [origin]
        self.radius = radius
        self.color = color

    def draw(self, ax):
        offset = (self.frames[0].absolute[0, 2], self.frames[0].absolute[1, 2])
        cc = Circle(offset, self.radius, fill=False, edgecolor=self.color)
        ax.add_artist(cc)
