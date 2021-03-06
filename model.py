"""

Author: Jack Minardi
Email: jminardi@seas.harvard.edu

"""
import math
import threading
import numpy as np

from mecode import G
from mecode.devices.efd_pressure_box import EFDPressureBox


class Model(object):

    def __init__(self, diameter=25, line_spacing=0.1, start=(0, 0),
                 num_anchors=8, anchor_width=6, heaven=2, layer_thickness=.15,
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

        self.dance_steps = []
        self.needs_cleaning = False

    def get_targets_y_spaced(self):
        a, b = self.start
        r = self.diameter / 2.0
        y = np.arange(0, -r, -self.line_spacing)
        x = np.sqrt(r ** 2 - y ** 2) + a
        y += b
        targets_x = np.hstack(((-1 * x) + 2 * a, x[::-1]))
        targets_y = np.hstack((y, y[::-1]))
        return np.array([targets_x, targets_y]).T

    def get_anchor_idxs(self, shift=0):
        targets = self.get_targets_y_spaced()
        anchors_idx = np.linspace(0, len(targets), self.num_anchors + 1)
        anchors_idx = np.round(anchors_idx[:-1])
        anchors_idx += (anchors_idx[1] - anchors_idx[0]) / 2
        anchors_idx += shift * 0.5 * self.anchor_width
        return anchors_idx.astype('int')

    def draw_bundles(self, shift):
        targets = self.get_targets_y_spaced()
        right_targets = targets[len(targets) / 2:]
        num_left_anchors = self.num_anchors / 2
        anchor_idxs = self.get_anchor_idxs(shift)
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
                    self.pre_dance(anchor, target)
                else:
                    self.pre_dance(target, anchor)
                tic *= -1
            g.set_valve(0, 0)

    def draw_bundles_right(self, shift):
        targets = self.get_targets_y_spaced()
        left_targets = targets[:len(targets) / 2]
        num_right_anchors = self.num_anchors / 2
        anchor_idxs = self.get_anchor_idxs(shift)
        z = self.layer_thickness
        tic = 1
        for i in range(self.num_anchors / 2):
            for j in range(len(left_targets) / num_right_anchors):
                offset = (j % self.anchor_width) - (self.anchor_width / 2)
                target_idx = ((j * num_right_anchors) - i)
                anchor_idx = (anchor_idxs[i + num_right_anchors] - offset) % len(targets)
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
                    self.pre_dance(anchor, target)
                else:
                    self.pre_dance(target, anchor)
                tic *= -1
            g.set_valve(0, 0)

    def pre_dance(self, fro, to):
        """ Ramp up for the dance.
        """
        self.dance_steps.append((fro, to))

    def dance(self, fro, to):
        """ Perform the ritual dance to appease the PECUU gods.
        This is the actual movement procedure.
        """
        to, fro = list(to), list(fro)
        z_dim = self.z_dim
        heaven = self.heaven
        rway = self.runway if to[0] > fro[0] else -self.runway
        gnd_spd = self.ground_speed
        air_spd = self.air_speed

        # The following lines are a HACK
        fro_level = math.floor(fro[2] / self.layer_thickness)
        to_level = math.floor(to[2] / self.layer_thickness)
        fro[0] -= fro_level * rway
        to[0] += to_level * rway
        to[2], fro[2] = (0.6 * self.layer_thickness, 0.6 * self.layer_thickness)

        # This moves right above the first point
        g.abs_move(fro[0] - rway, fro[1], **{z_dim: fro[2] + heaven})
        g.feed(air_spd)
        g.move(**{z_dim: -heaven})  #this move drops from the air down to the first point

        if self.needs_cleaning is True:
            return

        g.set_valve(0, 1)
        g.feed(gnd_spd)
        g.move(rway)  #this moves along the ground to build the "runway"
        g.dwell(self.stamp_time)

        if self.needs_cleaning is True:
            return

        # These moves are the actual filament
        g.move(**{z_dim: heaven})
        g.feed(air_spd)
        g.abs_move(x=to[0], y=to[1], **{z_dim: to[2] + heaven})

        if self.needs_cleaning is True:
            return

        # Drop down at the end point of the filament
        g.move(**{z_dim: -heaven})
        g.feed(gnd_spd)

        if self.needs_cleaning is True:
            return

        g.move(rway)  # draw the last runway
        g.set_valve(0, 0)

        if self.needs_cleaning is True:
            return

        g.dwell(self.stamp_time)
        g.move(**{z_dim: heaven})  # go back up to a safe height
        g.feed(air_spd)

    def draw_and_listen(self):
        """ Master controller that checks for requested cleaning, and otherwise
        calls the dance functions for each filament.
        """
        total_steps = len(self.dance_steps)
        current_step = 0
        while current_step < total_steps:
            if self.needs_cleaning is True:
                self.move_to_clean()
                self.needs_cleaning = False
                current_step -= 1
            fro, to = self.dance_steps[current_step]
            self.dance(fro, to)
            current_step += 1

    def start_thread(self):
        self._thread = threading.Thread(target=self.draw_and_listen)
        self._thread.start()

    def clean(self):
        self.needs_cleaning = True

    def move_to_clean(self):
        g.set_valve(0, 0)
        g.feed(200)
        g.abs_move(**{self.z_dim: 45.2})
        g.abs_move(x=283, y=13)
        g.abs_move(**{self.z_dim: 10})
        sign = 1
        g.feed(500)
        for _ in range(100):
            g.move(x=sign * 2)
            sign *= -1
        for _ in range(100):
            g.move(y=sign * 2)
            sign *= -1
        g.feed(200)
        g.abs_move(**{self.z_dim: 45.2})
        g.move(x=50)

    def draw_linear(self, z=0):
        """ Only used with PDMS
        """
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
        """ Only used with PDMS
        """
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
        """ This function computes the bundle positions and needed dance steps,
        it does not actaully print.
        """
        if style == 'linear':
            draw = self.draw_linear
        elif style == 'arc':
            draw = self.draw_basic_arcs
        elif style == 'bundles':
            for i in range(num):
                if i % 2 == 0:
                    self.draw_bundles(i/2)
                else:
                    self.draw_bundles_right(i/2)
            return
        height = self.layer_thickness
        for z_height in np.arange(height, height * (num + 1), height):
            draw(z=z_height)


if __name__ == '__main__':


    x = 0  # change these to 0 when testing
    y = 0  # change these to 0 when testing
    z = 47.962270
    speed = 6
    pressure = 70
    num_layers = 30

    g = G(
        #outfile=r"C:\Users\Lewis Group\Documents\GitHub\out.pgm",  # valid on robomama printer computer
        outfile=r'/Users/jack/Desktop/test.gcode',  # this is where the GCode file is written
        print_lines=False,
        direct_write=True,  # silence for testing
    )
    model = Model(
        z_dim='z',  # change this to 'z' when testing, and 'A' when printing
        line_spacing=0.03,
        diameter=15,
        layer_thickness=0.007,
        start=(x, y),
        stamp_time=0.1,
        heaven=0.15,
        runway=0.2,
        ground_speed=2,
        air_speed=8,
    )
    #pb = EFDPressureBox('COM4')  #silence this when testing

    abs_0 = z
    setpt = abs_0 - 4

    g.write('POSOFFSET CLEAR X Y A B')
    g.feed(20)
    g.abs_move(A=-setpt)
    g.set_home(A=abs_0 - setpt)
    #pb.set_pressure(pressure)  #silence this when testing
    #pb.toggle_pressure()  #silence this when testing
    x, y = model.get_targets_y_spaced()[model.get_anchor_idxs()[0]]
    g.abs_move(x, y, **{model.z_dim: model.heaven * 2})

    g.feed(speed)

    model.draw_layers('bundles', 30)
    model.start_thread()

    #model.draw_and_listen()

    #g.set_valve(0, 0)
    #pb.set_pressure(0)
    #g.move(X=50, Y=50, A=30)


    #pb.toggle_pressure()
    #pb.disconnect()
    #g.teardown()
