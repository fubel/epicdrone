from __future__ import division
from __future__ import print_function

import csv


class Marker(object):
    def __init__(self, marker_id, position, side):
        """
        Constructor
        Args:
            position: the landmark position in global coordinates
            side: 'N', 'W', 'E' or 'S'
        """
        self.id = marker_id
        self.position = position
        self.side = side


    @staticmethod
    def read(filename):
        """
        Reads a list of markers from csv file:

        Args:
            filename: The file containing the markers

        Returns:
            A marker dict (todo: should be list in the end)

        """
        with open(filename) as file:
            markers = {}
            csv_reader = csv.reader(file, delimiter=';')
            for row in csv_reader:
                markers[int(row['id'])] = [[float(row['position x']),
                                            float(row['position y']),
                                            float(row['position z'])],
                                            [0.,0.,0.], row['side']]
        return markers