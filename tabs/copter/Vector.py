import math

__author__ = 'CopterLab'


class Vector(tuple):
    @property
    def x(self):
        return self[0]
    @property
    def y(self):
        return self[1]
    @property
    def z(self):
        return self[2]

    def __add__(self, a):
        return Vector(x+y for x, y in zip(self, a))

    def __sub__(self, a):
        return Vector(x-y for x, y in zip(self, a))

    def __mul__(self, c):
        return Vector(x*c for x in self)

    def magnitude(self):
        return math.sqrt(self[0] * self[0] + self[1] * self[1] + self[2] * self[2])

