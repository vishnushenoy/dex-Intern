from __future__ import print_function


class swapp:
    def __init__(self, x, y, time):
        self._x = x
        self._y = y
        self._time = time

    def getx(self):
        return self._x

    def gety(self):
        return self._y

    def gettime(self):
        return self._time

    def getNode(self):
        return [self._x, self._y, self._time]

    def getxy(self):
        return (self._x, self._y)