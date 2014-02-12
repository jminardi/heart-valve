"""
This code outputs the toolpath to create a heart valv
"""
import numpy as np

from mecode import MeCode


class HeartValveModel(object):

    def __init__(self, diameter=25, line_spacing=0.1, start=(0, 0),
                 num_anchors=8, anchor_width=10, heaven=2, layer_thicknes=.15,
                 arc_radius=100):
        """
        Parameters
        ----------
        diameter : float
            Diameter of the valve flap in mm.
        line_spacing : float
            Diameter of the nozzel in mm.
        start : tuple of floats (len = 2)
            starting position in (x, y)
        num_anchors : int
            Total number of anchors. Should be a multiple of two.
        anchor_width : float
            Width of the bundle base in multiples of the line_spacing.
        heaven : float
            Safe height to raise to to clear all features.
        arc_radius : float
            Radius of the connecting arcs.
        """
        self.diameter = diameter
        self.line_spacing = line_spacing
        self.start = start
        self.num_anchors = num_anchors
        self.anchor_width = anchor_width
        self.heaven = heaven
        self.layer_thicknes = layer_thicknes
        self.arc_radius = arc_radius

        self.circum = np.pi * diameter
        self.z_heights = (np.zeros(len(self.get_targets_y_spaced())) +
                          layer_thicknes)

    def get_targets_y_spaced(self):
        a, b = self.start
        r = self.diameter / 2.0
        y = np.arange(0, -r, -self.line_spacing)
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
        r = self.arc_radius
        z = self.layer_thicknes
        heaven = self.heaven
        tic = 1
        for i in range(self.num_anchors / 2):
            for j in range(len(right_targets) / num_left_anchors):
                offset = (j % self.anchor_width) - (self.anchor_width / 2)
                target_idx = ((j * num_left_anchors) - i) + (len(targets) / 2)
                anchor_idx = anchor_idxs[i] - offset
                if target_idx < (len(targets) / 2):
                    continue
                target = targets[target_idx]
                anchor = targets[anchor_idx]
                target_z = self.z_heights[target_idx]
                self.z_heights[target_idx] += z
                anchor_z = self.z_heights[anchor_idx]
                self.z_heights[anchor_idx] += z
                if tic == 1:
                    g.abs_move(anchor[0], anchor[1], z=anchor_z + heaven)
                    g.clip('z', '+x', -heaven)
                    g.set_valve(0, 1)
                    g.abs_arc(direction='CCW', radius=r,
                              x=target[0], y=target[1], z=target_z)
                    g.set_valve(0, 0)
                    g.clip('z', '+x', heaven)
                else:
                    g.abs_move(target[0], target[1], z=target_z + heaven)
                    g.clip('z', '-x', -heaven)
                    g.set_valve(0, 1)
                    g.abs_arc(direction='CW', radius=r,
                              x=anchor[0], y=anchor[1], z=anchor_z)
                    g.set_valve(0, 0)
                    g.clip('z', '-x', heaven)
                tic *= -1

    def draw_bundles_right(self, z=None):
        targets = self.get_targets_y_spaced()
        left_targets = targets[:len(targets) / 2]
        num_right_anchors = self.num_anchors / 2
        anchor_idxs = self.get_anchor_idxs()
        r = self.arc_radius
        z = self.layer_thicknes
        heaven = self.heaven
        tic = 1
        for i in range(self.num_anchors / 2):
            for j in range(len(left_targets) / num_right_anchors):
                offset = (j % self.anchor_width) - (self.anchor_width / 2)
                target_idx = ((j * num_right_anchors) - i)
                anchor_idx = anchor_idxs[i + num_right_anchors] - offset
                if target_idx < 0 or anchor_idx < 0:
                    continue
                target = targets[target_idx]
                anchor = targets[anchor_idx]
                target_z = self.z_heights[target_idx]
                self.z_heights[target_idx] += z
                anchor_z = self.z_heights[anchor_idx]
                self.z_heights[anchor_idx] += z
                if tic == 1:
                    g.abs_move(anchor[0], anchor[1], z=anchor_z + heaven)
                    g.clip('z', '-x', -heaven)
                    g.set_valve(0, 1)
                    g.abs_arc(direction='CW', radius=r,
                              x=target[0], y=target[1], z=target_z)
                    g.set_valve(0, 0)
                    g.clip('z', '-x', heaven)
                else:
                    g.abs_move(target[0], target[1], z=target_z + heaven)
                    g.clip('z', '+x', -heaven)
                    g.set_valve(0, 1)
                    g.abs_arc(direction='CCW', radius=r,
                              x=anchor[0], y=anchor[1], z=anchor_z)
                    g.set_valve(0, 0)
                    g.clip('z', '+x', heaven)
                tic *= -1

    def draw_linear(self, z=0):
        targets = self.get_targets_y_spaced()
        left_targets = targets[:len(targets) / 2]
        right_targets = targets[len(targets) / 2:]
        heaven = self.heaven
        g.abs_move(x=left_targets[0][0], y=left_targets[0][1], z=z + heaven)
        g.clip('z', '+x', -heaven)
        g.set_valve(0, 1)
        tic = 1
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
        heaven = self.heaven
        g.abs_move(x=left_targets[0][0], y=left_targets[0][1], z=z + heaven)
        g.clip('z', '+x', -heaven)
        g.set_valve(0, 1)
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
        g.set_valve(0, 0)
        g.clip('z', '-y', heaven)

    def draw_layers(self, type='linear', num=5):
        if type == 'linear':
            draw = self.draw_linear
        elif type == 'arc':
            draw = self.draw_basic_arcs
        elif type == 'bundles':
            draw = self.draw_bundles
        height = self.layer_thicknes
        for z_height in np.arange(height, height * (num + 1), height):
            draw(z=z_height)


if __name__ == '__main__':
    g = MeCode(
        #outfile=r"C:\Users\Lewis Group\Documents\GitHub\heart-valve\out.pgm",
        #print_lines=False
    )
    valve = HeartValveModel(
        #line_spacing=.18,
        #diameter=40,
        #start=(373, 70),
    )
    abs_0 = 79.141
    g.feed(20)
    #g.abs_move(A=-5)
    #g.set_home(A=abs_0 - 5)
    g.set_valve(0, 0)
    g.set_pressure(9, 55)
    g.toggle_pressure(9)
    g.feed(12)

    valve.draw_bundles()
    valve.draw_bundles_right()

    g.toggle_pressure(9)
    g.teardown()
