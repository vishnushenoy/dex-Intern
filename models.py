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


class package:
    def __init__(self, demand_loc, supply_loc, package_id):
        self._demand_loc = demand_loc
        self._supply_loc = supply_loc
        self._package_id = package_id

    def getdemandloc(self):
        return self._demand_loc

    def getsupplyloc(self):
        return self._supply_loc

    def getpackageid(self):
        return self._package_id

    def getpackagedetails(self):
        return [self._supply_loc, self._demand_loc, self._package_id]

    def getsupplyanddemand(self):
        return [self._supply_loc, self._demand_loc]