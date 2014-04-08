"""
This code outputs the toolpath to create a heart valve

Author: Jack Minardi
Email: jminardi@seas.harvard.edu

"""
import math
import numpy as np

from mecode import G


class HeartValveModel(object):

    def __init__(self, diameter=25, line_spacing=0.1, start=(0, 0),
                 num_anchors=8, anchor_width=10, heaven=2, layer_thickness=.15,
                 arc_radius=100, z_dim='z', stamp_time=0.5, runway=3,
                 ground_speed=0.5, air_speed=1):
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
        layer_thickness : float
            Height to raise in Z between layers.
        arc_radius : float
            Radius of the connecting arcs.
        runway : float
            Length in mm of the extra legs used to ensure stick.
        """
        self.diameter = diameter
        self.line_spacing = line_spacing
        self.start = start
        self.num_anchors = num_anchors
        self.anchor_width = anchor_width
        self.heaven = heaven
        self.layer_thickness = layer_thickness
        self.arc_radius = arc_radius
        self.stamp_time = stamp_time
        self.runway = runway
        self.ground_speed = ground_speed
        self.air_speed = air_speed

        self.circum = np.pi * diameter
        self.z_heights = (np.zeros(len(self.get_targets_y_spaced())) +
                          (0.60 * layer_thickness))
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
        z = self.layer_thickness
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
                anchor_z = self.z_heights[anchor_idx]
                target = target[0], target[1], target_z
                anchor = anchor[0], anchor[1], anchor_z
                self.z_heights[target_idx] += z
                self.z_heights[anchor_idx] += z
                if tic == 1:
                    self.dance(anchor, target)
                else:
                    self.dance(target, anchor)
                tic *= -1
            g.set_valve(0, 0)

    def draw_bundles_right(self):
        targets = self.get_targets_y_spaced()
        left_targets = targets[:len(targets) / 2]
        num_right_anchors = self.num_anchors / 2
        anchor_idxs = self.get_anchor_idxs()
        z = self.layer_thickness
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
                anchor_z = self.z_heights[anchor_idx]
                target = target[0], target[1], target_z
                anchor = anchor[0], anchor[1], anchor_z
                self.z_heights[target_idx] += z
                self.z_heights[anchor_idx] += z
                if tic == 1:
                    self.dance(anchor, target)
                else:
                    self.dance(target, anchor)
                tic *= -1
            g.set_valve(0, 0)

    def dance(self, fro, to):
        """ Perform the ritual dance to appease the pHEMA gods.
        """
        to, fro = list(to), list(fro)
        z_dim = self.z_dim
        heaven = self.heaven
        rway = self.runway if to[0] > fro[0] else -self.runway
        gnd_spd = self.ground_speed
        air_spd = self.air_speed
        
        fro_level = math.floor(fro[2] / self.layer_thickness)
        to_level = math.floor(to[2] / self.layer_thickness)
        fro[0] -= fro_level * rway
        to[0] += to_level * rway
        to[2], fro[2] = (0.6 * self.layer_thickness, 0.6 * self.layer_thickness)

        g.abs_move(fro[0] - rway, fro[1], **{z_dim: fro[2] + heaven})
        g.move(**{z_dim: -heaven})

        g.set_valve(0, 1)
        g.feed(gnd_spd)
        g.move(rway)
        g.dwell(self.stamp_time)
        
        g.move(**{z_dim: heaven})
        g.feed(air_spd)
        g.abs_move(x=to[0], y=to[1], **{z_dim: to[2] + heaven})
        
        g.move(**{z_dim: -heaven})
        g.feed(gnd_spd)
        g.move(rway)
        g.set_valve(0, 0)
        g.dwell(self.stamp_time)
        g.move(**{z_dim: heaven})
        g.feed(air_spd)

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
        height = self.layer_thickness
        for z_height in np.arange(height, height * (num + 1), height):
            draw(z=z_height)


if __name__ == '__main__':
    g = G(
        outfile=r"C:\Users\Lewis Group\Documents\GitHub\heart-valve\out.pgm",
        #outfile=r'/Users/jack/Desktop/test.gcode',
        print_lines=False,
    )
    valve = HeartValveModel(
        z_dim='A',
        line_spacing=0.03,
        diameter=25,
        layer_thickness=0.008,
        start=(418.88, 109.08),
        stamp_time=0.1,
        heaven=0.2,
        runway=0.7,
    )
    abs_0 = 50.475739
    #g.setup()
    g.feed(20)
    g.abs_move(A=-45)
    g.set_home(A=abs_0 - 45)
    g.set_pressure(4, 50)
    g.toggle_pressure(4)

    x, y = valve.get_targets_y_spaced()[valve.get_anchor_idxs()[0]]
    g.abs_move(x, y, **{valve.z_dim: valve.heaven * 2})

    g.feed(4.5)
    valve.draw_layers('bundles', 1)
    g.set_valve(0, 0)
    g.set_pressure(4, 0)
    g.move(X=50, Y=50, A=30)
    
    
    g.toggle_pressure(4)
    g.teardown()
