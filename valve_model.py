"""
This code outputs the toolpath to create a heart valve

Author: Jack Minardi
Email: jminardi@seas.harvard.edu

"""
import numpy as np

from mecode import MeCode


class HeartValveModel(object):

    def __init__(self, diameter=25, line_spacing=0.1, start=(0, 0),
                 num_anchors=8, anchor_width=10, heaven=2, layer_thicknes=.15,
                 arc_radius=100, z_dim='z', stamp_time=0.5):
        """
        Parameters
        ----------
        diameter : float
            Diameter of the valve flap in mm.
        line_spacing : float
            Spacing between line end points.
        start : tuple of floats (len = 2)
            starting position in (x, y)
        num_anchors : int
            Total number of anchors. Should be a multiple of two.
        anchor_width : float
            Width of the bundle base in multiples of the line_spacing.
        heaven : float
            Safe height to raise to to clear all features.
        layer_thicknes : float
            Height to raise in Z between layers.
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
        self.stamp_time = stamp_time

        self.circum = np.pi * diameter
        self.z_heights = (np.zeros(len(self.get_targets_y_spaced())) +
                          (0.60 * layer_thicknes))
        self.z_dim = z_dim

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

    def draw_bundles(self):
        targets = self.get_targets_y_spaced()
        right_targets = targets[len(targets) / 2:]
        num_left_anchors = self.num_anchors / 2
        anchor_idxs = self.get_anchor_idxs()
        r = self.arc_radius
        z = self.layer_thicknes
        heaven = self.heaven
        tic = 1
        z_dim = self.z_dim
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
                    g.abs_move(anchor[0], anchor[1],
                            **{z_dim: anchor_z + heaven})
                    g.abs_move(**{z_dim: anchor_z})
                    g.dwell(self.stamp_time)
                    g.move(**{z_dim: heaven})
                    #g.clip('A', '-x', -heaven)
                    #g.move(A=-heaven)
                    g.abs_move(#direction='CCW', #radius=r,
                               x=target[0], y=target[1],
                               **{z_dim: target_z + heaven})
                    g.abs_move(**{z_dim: target_z})
                    g.dwell(self.stamp_time)
                    g.move(**{z_dim: heaven})
                    #g.set_valve(0, 0)
                    #g.clip('A', '+x', heaven)
                    #g.move(A=heaven)
                else:
                    g.abs_move(target[0], target[1], **{z_dim: target_z})
                    g.dwell(self.stamp_time)
                    g.move(**{z_dim: heaven})
                    #g.clip('A', '+x', -heaven)
                    #g.move(A=-heaven)
                    g.set_valve(0, 1)
                    g.abs_move(#direction='CW', #radius=r,
                               x=anchor[0], y=anchor[1],
                               **{z_dim: anchor_z + heaven})
                    g.abs_move(**{z_dim: anchor_z})
                    g.dwell(self.stamp_time)
                    g.move(**{z_dim: heaven})
                    g.set_valve(0, 1)
                    #g.set_valve(0, 0)
                    #g.clip('A', '-x', heaven)
                    #g.move(A=heaven)
                tic *= -1
            g.set_valve(0, 0)

    def draw_bundles_right(self):
        targets = self.get_targets_y_spaced()
        left_targets = targets[:len(targets) / 2]
        num_right_anchors = self.num_anchors / 2
        anchor_idxs = self.get_anchor_idxs()
        r = self.arc_radius
        z = self.layer_thicknes
        heaven = self.heaven
        z_dim = self.z_dim
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
                    g.abs_move(anchor[0], anchor[1],
                               **{z_dim: anchor_z + heaven})
                    g.abs_move(**{z_dim: anchor_z})
                    g.dwell(self.stamp_time)
                    g.move(**{z_dim: heaven})
                    #g.clip('A', '+x', -heaven)
                    #g.move(A=-heaven)
                    g.set_valve(0, 1)
                    g.abs_move(#direction='CW', #radius=r,
                               x=target[0], y=target[1] ,
                               **{z_dim: target_z + heaven})
                    g.abs_move(**{z_dim: target_z})
                    g.dwell(self.stamp_time)
                    g.move(**{z_dim: heaven})
                    #g.set_valve(0, 0)
                    #g.clip('A', '-x', heaven)
                    #g.move(A=heaven)
                else:
                    g.abs_move(target[0], target[1],
                               **{z_dim: target_z + heaven})
                    g.abs_move(**{z_dim: target_z})
                    g.dwell(self.stamp_time)
                    g.move(**{z_dim: heaven})
                    #g.clip('A', '-x', -heaven)
                    #g.move(A=-heaven)
                    g.set_valve(0, 1)
                    g.abs_move(#direction='CCW', #radius=r,
                               x=anchor[0], y=anchor[1],
                               **{z_dim: anchor_z + heaven})
                    g.abs_move(**{z_dim: anchor_z})
                    g.dwell(self.stamp_time)
                    g.move(**{z_dim: heaven})
                    #g.set_valve(0, 0)
                    #g.clip('A', '+x', heaven)
                    #g.move(A=heaven)
                tic *= -1
            g.set_valve(0, 0)

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
        g.clip('z', '-x', -heaven)
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

    def draw_layers(self, style='linear', num=5):
        if style == 'linear':
            draw = self.draw_linear
        elif style == 'arc':
            draw = self.draw_basic_arcs
        elif style == 'bundles':
            for i in range(num):
                if i % 2 == 0:
                    self.draw_bundles()
                else:
                    self.draw_bundles_right()
            return
        height = self.layer_thicknes
        for z_height in np.arange(height, height * (num + 1), height):
            draw(z=z_height)


if __name__ == '__main__':
    #cal_data = np.array([[395, 128, 0], [395, 65, 0], [476, 65, -0.030], [476, 128, -0.030]])
    g = MeCode(
        #outfile=r"C:\Users\Lewis Group\Documents\GitHub\heart-valve\out.pgm",
        outfile=r'/Users/jack/Desktop/out.gcode',
        print_lines=False,
        #cal_data=cal_data
    )
    valve = HeartValveModel(
        z_dim='z',
        line_spacing=0.04,
        diameter=25,
        layer_thicknes=0.008,
        #start=(444.69, 95.645),
        stamp_time=0.8,
        heaven=1,
    )
    g.setup()
    abs_0 = 51.12955 + (0.003833*2)
    g.feed(20)
    g.abs_move(A=-45)
    g.set_home(A=abs_0 - 45)
    g.set_valve(0, 0)
    g.set_pressure(4, 44)
    g.toggle_pressure(4)
    g.feed(5)

    g.set_valve(0, 1)
    x, y = valve.get_targets_y_spaced()[valve.get_anchor_idxs()[0]]
    g.abs_move(x - 5, y, A=valve.z_heights[0])
    g.move(5)
    valve.draw_layers('bundles', 1)

    g.toggle_pressure(4)
    g.teardown()
