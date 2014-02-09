import numpy as np

from mecode import MeCode

g = MeCode()


class HeartValveModel(object):

    def __init__(self, diameter=25, nozzle_diameter=0.1, start=(0, 0),
                 num_anchors=8, anchor_width=10):
        """
        Parameters
        ----------
        diameter : float
            Diameter of the valve flap in mm.
        nozzle_diameter : float
            Diameter of the nozzel in mm.
        start : tuple of floats (len = 2)
            starting position in (x, y)
        num_anchors : int
            Total number of anchors. Should be a multiple of two.
        anchor_width : float
            Width of the bundle base in multiples of the nozzle_diameter.
        """
        self.diameter = diameter
        self.nozzle_diameter = nozzle_diameter
        self.start = start
        self.num_anchors = num_anchors
        self.anchor_width = anchor_width

        self.circum = np.pi * diameter
        self.z_height = np.zeros(len(self.get_targets()))

    def get_targets(self):
        num_segments = round_multiple(self.circum / self.nozzle_diameter) / 2
        r = self.diameter / 2.0
        theta = np.linspace(np.pi, 2 * np.pi, num_segments)
        x = (r * np.cos(theta)) + self.start[0]
        y = (r * np.sin(theta)) + self.start[1]
        return np.array([x, y]).T

    def get_anchor_idxs(self):
        targets = self.get_targets()
        anchors_idx = np.linspace(0, len(targets), self.num_anchors + 1)
        anchors_idx = np.round(anchors_idx[:-1])
        anchors_idx += (anchors_idx[1] - anchors_idx[0]) / 2
        return anchors_idx.astype('int')

    def get_anchors(self):
        targets = self.get_targets()
        anchors = targets[self.get_anchor_idxs()]
        return anchors

    def draw_from_anchors(self):
        targets = self.get_targets()
        anchors = self.get_anchors()
        right_targets = targets[len(targets) / 2:]
        left_anchors = anchors[:len(anchors) / 2]
        num_left_anchors = self.num_anchors / 2
        tic = 1
        anchor_idxs = self.get_anchor_idxs()
        for i in range(len((left_anchors))):
            for j in range(len(right_targets) / num_left_anchors):
                offset = (j % self.anchor_width) - (self.anchor_width / 2)
                target_idx = ((j * num_left_anchors) - i) + (len(targets) / 2)
                anchor_idx = anchor_idxs[i] - offset
                target = targets[target_idx]
                anchor = targets[anchor_idx]
                target_z = self.z_height[target_idx]
                self.z_height[target_idx] += 1 * self.nozzle_diameter
                anchor_z = self.z_height[anchor_idx]
                self.z_height[anchor_idx] += 1 * self.nozzle_diameter
                if tic == 1:
                    g.abs_move(anchor[0], anchor[1], z=anchor_z)
                    g.abs_arc(direction='CCW', radius=20,
                              x=target[0], y=target[1], z=target_z)
                else:
                    g.abs_move(target[0], target[1], z=target_z)
                    g.abs_arc(direction='CW', radius=20,
                              x=anchor[0], y=anchor[1], z=anchor_z)
                tic *= -1


def round_multiple(x, multiple=4.0):
    """ Round to the closest integer multiple of the given `multiple`.
    """
    return np.round(x / multiple) * multiple


if __name__ == '__main__':
    valve = HeartValveModel()
    valve.draw_from_anchors()
