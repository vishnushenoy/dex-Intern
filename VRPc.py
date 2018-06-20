"""Capacitated Vehicle Routing Problem"""

from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import kdtree


class Vehicle:                                # Problem Data Definition
    """Stores the property of a vehicle"""
    def __init__(self):
        """Initializes the vehicle properties"""
        self._capacity = 15

    @property
    def capacity(self):
        """Gets vehicle capacity"""
        return self._capacity


class CityBlock:
    """City block definition"""
    @property
    def width(self):
        """Gets Block size West to East"""
        return 114

    @property
    def height(self):
        """Gets Block size North to South"""
        return 80


class DataProblem:
    """Stores the data for the problem"""
    def __init__(self):
        """Initializes the data for the problem"""
        self._vehicle = Vehicle()
        self._num_vehicles = 5

        # Locations in block unit
        locations = \
            [(4, 4),                                # depot
                (2, 0), (8, 0),                        # row 0
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
            [0,                                          # depot
             1, 1,                                       # row 0
             2, 4,
             2, 4,
             8, 8,
             1, 2,
             1, 2,
             4, 4,
             8, 8]
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


def manhattan_distance(position_1, position_2):                 # Problem Constraints
    """Computes the Manhattan distance between two points"""
    return (abs(position_1[0] - position_2[0]) +
            abs(position_1[1] - position_2[1]))


class CreateDistanceEvaluator(object):
    """Create callback to calculate distances between points."""
    def __init__(self, data):
        """Initialize distance array."""
        size = data.num_locations
        depot = 0
        self.matrix = {}

        for from_node in xrange(size):
            self.matrix[from_node] = {}
            for to_node in xrange(size):
                if from_node == depot or to_node == depot:
                    # Define the distance from the depot to any node to be 0.
                    self.matrix[from_node][to_node] = 0
                else:
                    x1 = data.locations[from_node][0]
                    y1 = data.locations[from_node][1]
                    x2 = data.locations[to_node][0]
                    y2 = data.locations[to_node][1]
                    self.matrix[from_node][to_node] = manhattan_distance((x1, y1), (x2, y2))

    def distance_evaluator(self, from_node, to_node):
        return self.matrix[from_node][to_node]



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
    routing.AddDimension(
        demand_evaluator,
        0,                                              # null capacity slack
        data.vehicle.capacity,                          # vehicle maximum capacity
        True,                                           # start accumulating to zero
        "Capacity")


class ConsolePrinter:                                   # Print
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
        total_dist = 0
        for vehicle_id in xrange(self.data.num_vehicles):
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_dist = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(
                    self.assignment.Value(self.routing.NextVar(index)))
                route_dist += manhattan_distance(
                    self.data.locations[node_index],
                    self.data.locations[next_node_index])
                route_load += self.data.demands[node_index]
                plan_output += ' {0} Loading({1}) -> '.format(node_index, route_load)
                if self.data.supply[node_index] != 0:
                    route_load -= self.data.supply[node_index]
                    plan_output += ' {0} Unloading({1}) -> '.format(node_index, route_load)
                index = self.assignment.Value(self.routing.NextVar(index))

            node_index = self.routing.IndexToNode(index)
            total_dist += route_dist
            plan_output += ' {0} End \n'.format(node_index)
            plan_output += 'Distance of the route: {0}m\n'.format(route_dist)
            plan_output += 'Load of the route: {0}\n'.format(route_load)
            print(plan_output)
        print('Total Distance of all routes: {0}m'.format(total_dist))


def main():                                         # Main
    """Entry point of the program"""
    # Instantiate the data problem.
    data = DataProblem()
    start_locations = [0, 3, 15, 16, 7]
    end_locations = [2, 15, 7, 11, 1]

    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, start_locations, end_locations)
    # Define weight of each edge
    distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    # Add Capacity constraint
    demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
    add_capacity_constraints(routing, data, demand_evaluator)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
         routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    for vehicle_nbr in range(data.num_vehicles):
        start_var = routing.NextVar(routing.Start(vehicle_nbr))
        for node_index in range(routing.Size(), routing.Size()+routing.vehicles()):
            start_var.RemoveValue(node_index)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    printer = ConsolePrinter(data, routing, assignment)
    printer.print()
    root = kdtree.create(data.locations)
    kdtree.visualize(root)
    ans = root.search_knn(point=(114, 482), k=3, dist=None)
    print(ans)


if __name__ == '__main__':
    main()
