import math
import numpy as np


class HeadingStateEst:
    """
    Class to collect and process measurements of the heading sensor.
    It saves a number of past measurement values, which are used to dampen noise.
    obj.process_magnetometer_data(magnetic_value_x,magnetic_value_y) returns the filtered estimated current heading in radians
    """

    def __init__(self, num_averaging_, angle_offset_):
        self.filterlength = num_averaging_
        self.lastValues = np.zeros(num_averaging_)
        self.angle_offset = angle_offset_
        self.sensor_maxima = [[-0.01, 0.01], [-0.01, 0.01]]
        self.sensor_offsets = [0, 0]
        self.sensor_amplitude = [0.001, 0.001]

    def set_sensor_offset(self):
        self.sensor_offsets[0] = (self.sensor_maxima[0][0] + self.sensor_maxima[0][1]) / 2
        self.sensor_offsets[1] = (self.sensor_maxima[1][0] + self.sensor_maxima[1][1]) / 2

        self.sensor_amplitude[0] = (self.sensor_maxima[0][1] - self.sensor_maxima[0][0]) / 2
        self.sensor_amplitude[1] = (self.sensor_maxima[1][1] - self.sensor_maxima[1][0]) / 2

    def process_magnetometer_data(self, magx, magy):
        # adjust magnetometer maxima offsets.
        if self.sensor_maxima[0][0] > magx:
            self.sensor_maxima[0][0] = magx
            self.set_sensor_offset()
        elif self.sensor_maxima[0][1] < magx:
            self.sensor_maxima[0][1] = magx
            self.set_sensor_offset()

        if self.sensor_maxima[1][0] > magy:
            self.sensor_maxima[1][0] = magy
            self.set_sensor_offset()
        elif self.sensor_maxima[1][1] < magy:
            self.sensor_maxima[1][1] = magy
            self.set_sensor_offset()

        # Move all stored elements one place over.
        for i in range(self.filterlength - 1):
            self.lastValues[i] = self.lastValues[i + 1]

        normx = (magx - self.sensor_offsets[0]) / self.sensor_amplitude[0]
        normy = (magy - self.sensor_offsets[1]) / self.sensor_amplitude[1]

        # Add the new measurement to the saved values
        self.lastValues[self.filterlength - 1] = np.arctan2(normy, normx) + self.angle_offset

        # Find resultant vector of all measurements
        res = [0, 0]
        for measurement in self.lastValues:
            res = [res[0] + math.cos(measurement), res[1] + math.sin(measurement)]
        # Find the angle of the resultant vector of all measurements (a.k.a. the average angle)
        average_angle = np.arctan2(res[1], res[0])

        # Bound output angles
        angle_out = bound_angle_zero_2pi(average_angle)

        # Return estimated angle
        return angle_out


def error_angle_unwrap(a, aref):
    """
    Function that returns the shortest angle from a to aref (both defined in radians)
    """

    # Make sure both inputs are mapped between 0 and 2*pi
    a = bound_angle_zero_2pi(a)
    aref = bound_angle_zero_2pi(aref)

    # Take the difference
    if aref > a:
        if aref - a < math.pi:
            e = aref - a
        else:
            e = aref - a - 2 * math.pi
    else:
        if a - aref < math.pi:
            e = aref - a
        else:
            e = aref - a + 2 * math.pi

    # Return value
    return e


def bound_angle_zero_2pi(a):
    """
    Function that returns the input bounded between 0 and 2*pi
    """
    while a < 0:
        a = a + 2 * math.pi
    while a > 2 * math.pi:
        a = a - 2 * math.pi
    return a
