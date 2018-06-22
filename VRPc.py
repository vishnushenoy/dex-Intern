"""Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from models import *

###########################
# Problem Data Definition #
###########################

path_route = []

class Vehicle():
    """Stores the property of a vehicle"""
    def __init__(self):
        """Initializes the vehicle properties"""
        self._capacity = 15
        # Travel speed: 5km/h to convert in m/min
        self._speed = 5 * 60 / 3.6

    @property
    def capacity(self):
        """Gets vehicle capacity"""
        return self._capacity

    @property
    def speed(self):
        """Gets the average travel speed of a vehicle"""
        return self._speed


class CityBlock():
    """City block definition"""
    @property
    def width(self):
        """Gets Block size West to East"""
        return 114

    @property
    def height(self):
        """Gets Block size North to South"""
        return 80


class DataProblem():
    """Stores the data for the problem"""
    def __init__(self):
        """Initializes the data for the problem"""
        self._vehicle = Vehicle()
        self._num_vehicles = 4

        # Locations in block unit
        locations = \
                [(4, 4), # depot
                 (2, 0), (8, 0), # row 0
                 (0, 1), (1, 1),
                 (5, 2), (7, 2),
                 (3, 3), (6, 3),
                 (5, 5), (8, 5),
                 (1, 6), (2, 6),
                 (3, 7), (6, 7),
                 (0, 8), (7, 8)]
        # locations in meters using the city block dimension
        city_block = CityBlock()
        self._locations = [(
            loc[0]*city_block.width,
            loc[1]*city_block.height) for loc in locations]

        self._depot = 0

        self._demands = \
            [0, # depot
             1, 1, # 1, 2
             2, 4, # 3, 4
             2, 4, # 5, 6
             8, 8, # 7, 8
             1, 2, # 9,10
             1, 2, # 11,12
             4, 4, # 13, 14
             8, 8] # 15, 16

        self._supply = \
            [0,
             0, 2,
             0, 0,
             0, 2,
             0, 0,
             4, 1,
             2, 3,
             5, 2,
             0, 1]

        self._time_windows = \
            [(0, 0),
             (75, 85), (75, 85), # 1, 2
             (60, 70), (45, 55), # 3, 4
             (0, 8), (50, 60), # 5, 6
             (0, 10), (10, 20), # 7, 8
             (0, 10), (75, 85), # 9, 10
             (85, 95), (5, 15), # 11, 12
             (15, 25), (10, 20), # 13, 14
             (45, 55), (30, 40)] # 15, 16

    @property
    def vehicle(self):
        """Gets a vehicle"""
        return self._vehicle

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def locations(self):
        """Gets locations"""
        return self._locations

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self.locations)

    @property
    def depot(self):
        """Gets depot location index"""
        return self._depot

    @property
    def demands(self):
        """Gets demands at each location"""
        return self._demands

    @property
    def supply(self):
        """Gets demands at each location"""
        return self._supply

    @property
    def time_windows(self):
        """Gets (start time, end time) for each locations"""
        return self._time_windows

#######################
# Problem Constraints #
#######################


def manhattan_distance(position_1, position_2):
    """Computes the Manhattan distance between two points"""
    return (abs(position_1[0] - position_2[0]) +
            abs(position_1[1] - position_2[1]))


class CreateDistanceEvaluator(object):
    """Creates callback to return distance between points."""
    def __init__(self, data):
        """Initializes the distance matrix."""
        self._distances = {}

        # precompute distance between location to have distance callback in O(1)
        for from_node in xrange(data.num_locations):
            self._distances[from_node] = {}
            for to_node in xrange(data.num_locations):
                if from_node == to_node:
                    self._distances[from_node][to_node] = 0
                else:
                    self._distances[from_node][to_node] = (
                        manhattan_distance(
                            data.locations[from_node],
                            data.locations[to_node]))

    def distance_evaluator(self, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return self._distances[from_node][to_node]


class CreateDemandEvaluator(object):
    """Creates callback to get demands at each location."""
    def __init__(self, data):
        """Initializes the demand array."""
        self._demands = data.demands

    def demand_evaluator(self, from_node, to_node):
        """Returns the demand of the current node"""
        del to_node
        return self._demands[from_node]


def add_capacity_constraints(routing, data, demand_evaluator):
    """Adds capacity constraint"""
    capacity = "Capacity"
    routing.AddDimension(
        demand_evaluator,
        0, # null capacity slack
        data.vehicle.capacity, # vehicle maximum capacity
        True, # start cumul to zero
        capacity)


class CreateTimeEvaluator(object):
    """Creates callback to get total times between locations."""

    @staticmethod
    def travel_time(data, from_node, to_node):
        """Gets the travel times between two locations."""
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time = manhattan_distance(
                data.locations[from_node],
                data.locations[to_node]) / data.vehicle.speed
        return travel_time

    def __init__(self, data):
        """Initializes the total time matrix."""
        self._total_time = {}
        # precompute total time to have time callback in O(1)
        for from_node in xrange(data.num_locations):
            self._total_time[from_node] = {}
            for to_node in xrange(data.num_locations):
                if from_node == to_node:
                    self._total_time[from_node][to_node] = 0
                else:
                    self._total_time[from_node][to_node] = int(self.travel_time(data, from_node, to_node))

    def time_evaluator(self, from_node, to_node):
        """Returns the total time between the two nodes"""
        return self._total_time[from_node][to_node]


def add_time_window_constraints(routing, data, time_evaluator):
    """Add Global Span constraint"""
    time = "Time"
    horizon = 120
    routing.AddDimension(
        time_evaluator,
        horizon, # allow waiting time
        horizon, # maximum time per vehicle
        False, # don't force start cumul to zero since we are giving TW to start nodes
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    for location_idx, time_window in enumerate(data.time_windows):
        if location_idx == 0:
            continue
        index = routing.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # routing.AddToAssignment(time_dimension.SlackVar(index))
    for vehicle_id in xrange(data.num_vehicles):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data.time_windows[0][0], data.time_windows[0][1])
        # routing.AddToAssignment(time_dimension.SlackVar(index))

###########
# Printer #
###########


class ConsolePrinter():
    """Print solution to console"""
    def __init__(self, data, routing, assignment):
        """Initializes the printer"""
        self._data = data
        self._routing = routing
        self._assignment = assignment

    @property
    def data(self):
        """Gets problem data"""
        return self._data

    @property
    def routing(self):
        """Gets routing model"""
        return self._routing

    @property
    def assignment(self):
        """Gets routing model"""
        return self._assignment

    def print(self):
        """Prints assignment on console"""
        # Inspect solution.
        # capacity_dimension = self.routing.GetDimensionOrDie('Capacity')
        time_dimension = self.routing.GetDimensionOrDie('Time')
        total_dist = 0
        total_time = 0
        global path_route
        path_route = [list()]*self.data.num_vehicles
        # print(path_route)
        for vehicle_id in xrange(self.data.num_vehicles):
            path_route[vehicle_id] = []
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_dist = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(self.assignment.Value(self.routing.NextVar(index)))
                route_dist += manhattan_distance(self.data.locations[node_index], self.data.locations[next_node_index])
                # load_var = capacity_dimension.CumulVar(index)
                # route_load = self.assignment.Value(load_var)
                time_var = time_dimension.CumulVar(index)
                time_min = self.assignment.Min(time_var)
                time_max = self.assignment.Max(time_var)
                route_load += self.data.demands[node_index]
                swap=swapp(self.data.locations[node_index][0], self.data.locations[node_index][1], time_min)
                # print("Inputing (%s,%s) to %s" %(self.data.locations[node_index][0],self.data.locations[node_index][1],vehicle_id))
                path_route[vehicle_id].append(swap)
                
                # print("p", path_route)
                #print(swap.getx(),swap.gety(),swap.gettime())
                plan_output += ' {0} Loading({1})Time({2} {3}) -> '.format(node_index, route_load, time_min, time_max)
                if self.data.supply[node_index] != 0 and self.data.supply[node_index] < route_load:
                    route_load -= self.data.supply[node_index]
                    plan_output += ' {0} Unloading({1})Time({2} {3}) -> '.format(node_index, route_load, time_min, time_max)
                elif self.data.supply[node_index]>route_load:
                    route_load = 0
                    plan_output += ' {0} Unloading({1})Time({2} {3}) -> '.format(node_index, route_load, time_min, time_max)
                index = self.assignment.Value(self.routing.NextVar(index))
            # print("path route element of present vehicle")
            # for i in path_route[vehicle_id]:
            #     print(i.getxy())
            node_index = self.routing.IndexToNode(index)
            # load_var = capacity_dimension.CumulVar(index)
            # route_load = self.assignment.Value(load_var)
            time_var = time_dimension.CumulVar(index)
            route_time = self.assignment.Value(time_var)
            time_min = self.assignment.Min(time_var)
            time_max = self.assignment.Max(time_var)
            total_dist += route_dist
            total_time += route_time
            swap = swapp(self.data.locations[node_index][0], self.data.locations[node_index][1], time_min)
            path_route[vehicle_id].append(swap)
            #print(swap.getx(), swap.gety(), swap.gettime())
            plan_output += ' {0} Load({1}) Time({2},{3})\n'.format(node_index, route_load, time_min, time_max)
            plan_output += 'Distance of the route: {0} m\n'.format(route_dist)
            plan_output += 'Load of the route: {0}\n'.format(route_load)
            plan_output += 'Time of the route: {0} min\n'.format(route_time)
            print(plan_output)
        print('Total Distance of all routes: {0} m'.format(total_dist))
        print('Total Time of all routes: {0} min'.format(total_time))

########
# Main #
########


def main():
    """Entry point of the program"""
    # Instantiate the data problem.
    data = DataProblem()

    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    # Define weight of each edge
    distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    # Add Capacity constraint
    demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
    add_capacity_constraints(routing, data, demand_evaluator)
    # Add Time Window constraint
    time_evaluator = CreateTimeEvaluator(data).time_evaluator
    add_time_window_constraints(routing, data, time_evaluator)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    printer = ConsolePrinter(data, routing, assignment)
    printer.print()
    # for i in range(len(path_route)):
    #     print(path_route[i].getNode())
    

    l = []
    for x in path_route:
        l = []
        for y in x:
            if(y.getxy() != data.locations[data.depot]):
                l.append(y.getxy())
        print("Route : ",(l))
        print("Points not in present list : ")
        print((list(set(data.locations)-set(l)-set([data.locations[data.depot]]))))

    # tlist1 = [x.getxy() for x in path_route for y in x]
    # print(tlist1)

if __name__ == '__main__':
    main()
