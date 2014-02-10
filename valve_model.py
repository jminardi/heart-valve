"""
This code outputs the toolpath to create a heart valv
"""
import numpy as np

from mecode import MeCode


class HeartValveModel(object):

    def __init__(self, diameter=25, nozzle_diameter=0.1, start=(0, 0),
                 num_anchors=8, anchor_width=10, clip_height=2):
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
        clip_height : float
            Safe height to raise to to clear all features.
        """
        self.diameter = diameter
        self.nozzle_diameter = nozzle_diameter
        self.start = start
        self.num_anchors = num_anchors
        self.anchor_width = anchor_width
        self.clip_height = clip_height

        self.circum = np.pi * diameter
        self.z_height = np.zeros(len(self.get_targets_y_spaced()))

    def get_targets_y_spaced(self):
        a, b = self.start
        r = self.diameter / 2.0
        y = np.arange(0, -r, -self.nozzle_diameter)
        x = np.sqrt(r ** 2 - y ** 2) + a
        y += b
        targets_x = np.hstack(((-1 * x) + 2 * a, x[::-1]))
        targets_y = np.hstack((y, y[::-1]))
        return np.array([targets_x, targets_y]).T

    def get_anchor_idxs(self):
        targets = self.get_targets_y_spaced()
        anchors_idx = np.linspace(0, len(targets), self.num_anchors + 1)
        anchors_idx = np.round(anchors_idx[:-1])
        anchors_idx += (anchors_idx[1] - anchors_idx[0]) / 2
        return anchors_idx.astype('int')

    def get_anchors(self):
        targets = self.get_targets_y_spaced()
        anchors = targets[self.get_anchor_idxs()]
        return anchors

    def draw_bundles(self, z=None):
        targets = self.get_targets_y_spaced()
        right_targets = targets[len(targets) / 2:]
        num_left_anchors = self.num_anchors / 2
        anchor_idxs = self.get_anchor_idxs()
        tic = 1
        for i in range(self.num_anchors / 2):
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
                    g.abs_move(anchor[0], anchor[1], A=anchor_z)
                    g.abs_arc(direction='CCW', radius=20,
                              x=target[0], y=target[1], A=target_z)
                else:
                    g.abs_move(target[0], target[1], A=target_z)
                    g.abs_arc(direction='CW', radius=20,
                              x=anchor[0], y=anchor[1], A=anchor_z)
                tic *= -1

    def draw_linear(self, z=0):
        targets = self.get_targets_y_spaced()
        left_targets = targets[:len(targets) / 2]
        right_targets = targets[len(targets) / 2:]
        tic = 1
        heaven = self.clip_height
        g.abs_move(x=left_targets[0][0], y=left_targets[0][1], z=z + heaven)
        g.clip('z', '-y', -heaven)
        g.set_valve(0, 1)
        for left, right in zip(left_targets, right_targets[::-1]):
            if tic == 1:
                g.abs_move(x=left[0], y=left[1], z=z)
                g.abs_move(x=right[0], y=right[1], z=z)
            else:
                g.abs_move(x=right[0], y=right[1], z=z)
                g.abs_move(x=left[0], y=left[1], z=z)
            tic *= -1
        g.set_valve(0, 0)
        g.clip('z', '-y', heaven)

    def draw_basic_arcs(self, z=0):
        targets = self.get_targets_y_spaced()
        left_targets = targets[:len(targets) / 2]
        right_targets = targets[len(targets) / 2:]
        tic = 1
        for left, right in zip(left_targets, right_targets[::-1]):
            if tic == 1:
                g.abs_move(x=left[0], y=left[1], z=z)
                g.abs_arc(x=right[0], y=right[1], z=z, radius=self.diameter,
                          direction='CCW')
            else:
                g.abs_move(x=right[0], y=right[1], z=z)
                g.abs_arc(x=left[0], y=left[1], z=z, radius=self.diameter,
                          direction='CW')
            tic *= -1

    def draw_layers(self, type='linear', num=5, height='auto'):
        if type == 'linear':
            draw = self.draw_linear
        elif type == 'arc':
            draw = self.draw_basic_arcs
        elif type == 'bundles':
            draw = self.draw_bundles
        if height == 'auto':
            height = self.nozzle_diameter


if __name__ == '__main__':
    g = MeCode(
        #outfile=r"C:\Users\Lewis Group\Documents\GitHub\heart-valve\out.pgm",
        #print_lines=False
    )
    valve = HeartValveModel(
        nozzle_diameter=.18,
        diameter=40,
        #start=(373, 70),
    )
    abs_0 = 79.141
    g.feed(20)
    g.abs_move(A=-5)
    g.set_home(A=abs_0 - 5)
    g.set_valve(0, 0)
    g.set_pressure(9, 55)
    g.toggle_pressure(9)
    g.feed(12)
    valve.draw_linear()
    #for z in np.arange(0.15, .75, .15):
    #    valve.draw_linear(z)
    #    g.set_valve(0, 0)
    #    g.abs_move(A=2)
    g.toggle_pressure(9)
    g.teardown()
