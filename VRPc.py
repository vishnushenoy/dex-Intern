"""Capacitated Vehicle Routing Problem  (CVRP)"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from models import *
import kdtree

###########################
# Problem Data Definition #
###########################
path_route = []
cost = []
pack = []
route_dists = []
output=[]
class Vehicle:
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
        self._num_vehicles = 4

        # Locations in block unit
        locations = \
                [(4, 4), # depot
                 (2, 0), (8, 0),    # 1,2
                 (0, 1), (1, 1),    # 3,4
                 (5, 2), (7, 2),    # 5,6
                 (3, 3), (6, 3),    # 7,8
                 (5, 5), (8, 5),    # 9,10
                 (1, 6), (2, 6),    # 11,12
                 (3, 7), (6, 7),    # 13,14
                 (0, 8), (7, 8)]    # 15,16
        # locations in meters using the city block dimension
        city_block = CityBlock()
        self._locations = [(
            loc[0]*city_block.width,
            loc[1]*city_block.height) for loc in locations]

        self._depot = 0

        self._demands = \
            [0,  # depot
             1, 1,  # 1, 2
             2, 4,  # 3, 4
             2, 4,  # 5, 6
             8, 8,  # 7, 8
             1, 2,  # 9,10
             1, 2,  # 11,12
             4, 4,  # 13, 14
             8, 8]  # 15, 16

        self._supply = \
            [0,  # depot
             6, 2,  # 1,2
             6, 8,  # 3,4
             0, 8,  # 5,6
             0, 3,  # 7,8
             0, 8,  # 9,10
             3, 0,  # 11,12
             5, 0,  # 13,14
             0, 3]  # 15,16

        self._destinationLocationIndex=\
            [2, 0, 11,
             11, 1, 1,
             1, 1, 8,
             8, 0, 0,
             0, 0, 13,
             13, 13, 13,
             13, 3, 3,
             3, 6, 6,
             6, 6, 6,
             6, 6, 6,
             8, 2, 0,
             0, 4, 4,
             3, 3, 3,
             11, 16, 16,
             16, 10, 4,
             4, 4, 4,
             4, 4, 1,
             1, 10, 10,
             10, 10, 10,
             10, 10, 2]

        self._time_windows = \
            [(0, 0),
             (75, 85), (75, 85),  # 1, 2
             (60, 70), (45, 55),  # 3, 4
             (0, 8), (50, 60),  # 5, 6
             (0, 10), (10, 20),  # 7, 8
             (0, 10), (75, 85),  # 9, 10
             (85, 95), (5, 15),  # 11, 12
             (15, 25), (10, 20),  # 13, 14
             (45, 55), (30, 40)]  # 15, 16

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
    def destinationIndex(self):
        return self._destinationLocationIndex

    @property
    def time_windows(self):
        """Gets (start time, end time) for each locations"""
        return self._time_windows

    @property
    def total_demand(self):
        return sum([x for x in self._demands])

#######################
# Problem Constraints #
#######################

def comp(c_swap, c_noswap):
    if(c_swap > c_noswap):
        return False
    return True

def loc_node_to_index(node):
    data = DataProblem()
    return data.locations.index(node)


def index_to_loc_node(index):
    data=DataProblem()
    return data.locations[index]

def search_path(vehicle_nbr, index):
    location = index_to_loc_node(index)
    for i in range(len(path_route[vehicle_nbr])):
        if(path_route[vehicle_nbr][i].getxy() == location):
            return i

def find_path(node):
    data=DataProblem()
    for vehicle_nbr in range(data.num_vehicles):
        for i in range(len(path_route[vehicle_nbr])):
            if node == path_route[vehicle_nbr][i].getxy():
                return vehicle_nbr


def swap_decider(vehicle_nbr, neighbour , neighbour_index, c, locs):
    dec = {}
    neighbour_ind = []
    global cost
    out = []
    c_swap, c_noswap = 0, 0
    res1, res2 = [], []
    # for i in range(len(path_route[vehicle_nbr])):
    #     # print(data.locations.index((path_route[vehicle_nbr][i].getxy()[0], path_route[vehicle_nbr][i].getxy()[1])))
    #     # print(path_route[vehicle_nbr][i].getxy(),neighbour[neighbour_index])
    #     print("l",len(neighbour[neighbour_index]))
    #     for j in range(len(neighbour[neighbour_index])):
    #         vehicle_id = find_path(neighbour[neighbour_index][j])
    #         print(path_route[vehicle_id])
    for i in range(len(neighbour[neighbour_index])):
        neighbour_vehi=find_path(neighbour[neighbour_index][i])
        # print("n",loc_node_to_index(neighbour[neighbour_index][i]))
        # print(c)
        for j in c:
            # print("Case 1 : %d & %d , Distance : %d" %(vehicle_nbr, neighbour_vehi, route_dists[vehicle_nbr] + (2*j)))
            c_swap = route_dists[vehicle_nbr] + (2*j)
            # print(locs[neighbour_index])
            # print(loc_node_to_index(locs[neighbour_index]))
            s_index = search_path(vehicle_nbr,loc_node_to_index(locs[neighbour_index]))
            # print(s_index)
            # print("n",loc_node_to_index(neighbour[neighbour_index][i]))
            # print(s_index)
            res1 = []
            for k in range(s_index+1):
                res1.append(loc_node_to_index(path_route[vehicle_nbr][k].getxy()))
            res1.append(loc_node_to_index(neighbour[neighbour_index][i]))
            for k in range(s_index, len(path_route[vehicle_nbr])):
                res1.append(loc_node_to_index(path_route[vehicle_nbr][k].getxy()))

    # print(neighbour[neighbour_index][0])
    # node=(456,320)
    #print(vehicle_nbr, neighbour_vehi)
    for i in range(len(path_route[neighbour_vehi])):
      neighbour_ind.append(loc_node_to_index(path_route[neighbour_vehi][i].getxy()))
    path_package=temp(vehicle_nbr)
    # print(neighbour_ind)
    # print(path_package)
    for key in path_package:
      for val in path_package[key]:
        for i in range(1,len(neighbour_ind)-1):
          if(pack[val].getsupplylocindex()==neighbour_ind[i]):
            res2 = []
            dec[val] = neighbour_ind[i]
            print("\nCommon Package Id:{0} \n Source:{1} Destination:{2}".format(pack[val].getpackageid(), pack[val].getdemandlocindex(), pack[val].getsupplylocindex()))
            # print(pack[val].getsupplylocindex())
            # print(path_route[vehicle_nbr][-2].getxy())
            # print(index_to_loc_node(pack[val].getsupplylocindex()))
            # print("Case 2 : %d , Distance : %d" %(vehicle_nbr,route_dists[vehicle_nbr] + manhattan_distance(path_route[vehicle_nbr][-2].getxy(),index_to_loc_node(pack[val].getsupplylocindex()))))
            c_noswap = route_dists[vehicle_nbr] + manhattan_distance(path_route[vehicle_nbr][-2].getxy(),index_to_loc_node(pack[val].getsupplylocindex()))
            for k in range(len(path_route[vehicle_nbr])-1):
                res2.append(loc_node_to_index(path_route[vehicle_nbr][k].getxy()))
            res2.append(pack[val].getsupplylocindex())
            res2.append(loc_node_to_index(path_route[vehicle_nbr][-1].getxy()))
            if(comp(c_swap, c_noswap)):
                out = res1
            else:
                out = res2
    return out


def cost_fn_distance(vehicle_nbr, neighbour, path_route_index, neighbour_index):
    # print(type(path_route[vehicle_nbr][path_route_index].getxy()),type(neighbour[neighbour_index]))
    # print(len(neighbour[neighbour_index]))
    cost = []
    for i in range(len(neighbour[neighbour_index])):
        cost.append(manhattan_distance(path_route[vehicle_nbr][path_route_index].getxy(), neighbour[neighbour_index][i]))
    # print(cost)
    return cost


def temp(vehicle_nbr):
  destination_package = {}
  tem = []
  for i in range(len(path_route[vehicle_nbr])):
      tem.append(loc_node_to_index(path_route[vehicle_nbr][i].getxy()))
  for i in range(1,len(tem)-1):
        des = []
        for j in range(60):
          if(pack[j].getdemandlocindex()==tem[i]):
           des.append(j)
           #print(j,vars(pack[j]))
        destination_package[tem[i]] = des
  return destination_package


def nearest(point, Otherpoints):
    ansr = []
    # print(ansr)
    # input()
    points = list(set([point]))+Otherpoints
    ManhattanDistance = lambda a, b: sum(abs(a[axis]-b[axis]) for axis in range(len(a)))
    if Otherpoints!=[]:
        root = kdtree.create(points, dimensions=2)
        ans = root.search_knn(point=points[0], k=3, dist=ManhattanDistance)
        i = 0
        for r in ans:
          ansr.append(ans[i][0].data)
          i += 1
        return ansr[1:]


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


class CreateSupplyEvaluator(object):
    """Creates callback to get demands at each location."""
    def __init__(self, data):
        """Initializes the demand array."""
        self._supply = data.supply

    def supply_evaluator(self, from_node, to_node):
        """Returns the demand of the current node"""
        del from_node
        return self._supply[from_node]


def add_capacity_constraints(routing, data, demand_evaluator):
    """Adds capacity constraint"""
    capacity = "Capacity"
    routing.AddDimension(
        demand_evaluator,
        0,  # null capacity slack
        data.vehicle.capacity,  # vehicle maximum capacity
        True,  # start cumul to zero
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
        horizon,  # allow waiting time
        horizon,  # maximum time per vehicle
        False,  # don't force start cumul to zero since we are giving TW to start nodes
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
        global pack
        pack = [0]*self.data.total_demand
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
            total_load = 0
            id = 0
            for ind, pickup in enumerate(self.data.demands):
                # print(len(self.data.destinationIndex),self.data.destinationIndex[0])
                for i in range(pickup):
                    pack[id] = package(ind, self.data.destinationIndex[id], id)
                    id += 1
            # for i in pack:
            #     print(i.getpackagedetails())
            previous_node_index = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(self.assignment.Value(self.routing.NextVar(index)))
                # print(node_index, next_node_index)
                route_dist += manhattan_distance(self.data.locations[node_index], self.data.locations[next_node_index])
                # load_var = capacity_dimension.CumulVar(index)
                # route_load = self.assignment.Value(load_var)
                time_var = time_dimension.CumulVar(index)
                time_min = self.assignment.Min(time_var)
                time_max = self.assignment.Max(time_var)
                route_load += self.data.demands[node_index]
                total_load += self.data.demands[node_index]
                swap = swapp(self.data.locations[node_index][0], self.data.locations[node_index][1], time_min)
                # print("Inputing (%s,%s) to %s" %(self.data.locations[node_index][0],self.data.locations[node_index][1],vehicle_id))
                path_route[vehicle_id].append(swap)
                # print("p", path_route)
                # print(swap.getx(),swap.gety(),swap.gettime())
                plan_output += ' {0} Loading({1})Time({2} {3}) -> '.format(node_index, route_load, time_min, time_max)
                if self.data.supply[node_index] != 0 and self.data.supply[node_index] < route_load:
                    route_load -= self.data.supply[node_index]
                    plan_output += ' {0} Unloading({1})Time({2} {3}) -> '.format(node_index, route_load, time_min, time_max)
                    # for ind, i in enumerate(pack):
                    #     if(i.getdemandlocindex() == previous_node_index):
                    #         id = ind
                    #         while (i.getsupplylocindex() == 0):
                    #             for j in range(self.data.supply[node_index]):
                    #                 pack[id].setdestination(node_index)
                    #         id += 1
                    #     # print(i.getpackagedetails())
                elif self.data.supply[node_index] > route_load:
                    route_load = 0
                    plan_output += ' {0} Unloading({1})Time({2} {3}) -> '.format(node_index, route_load, time_min, time_max)
                index = self.assignment.Value(self.routing.NextVar(index))
                previous_node_index = node_index
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
            route_dists.append(route_dist)
            total_time += route_time
            swap = swapp(self.data.locations[node_index][0], self.data.locations[node_index][1], time_min)
            path_route[vehicle_id].append(swap)
            # destination = setdestination(path_route[vehicle_id][-2])
            # print(swap.getx(), swap.gety(), swap.gettime())
            plan_output += ' {0} Unloading({1}) Time({2},{3})\n'.format(node_index, route_load, time_min, time_max)
            plan_output += 'Distance of the route: {0} m\n'.format(route_dist)
            plan_output +=   'Load of the route: {0}\n'.format(total_load)
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
    supply_evaluator = CreateSupplyEvaluator(data).supply_evaluator
    add_capacity_constraints(routing, data, demand_evaluator)
    add_capacity_constraints(routing, data, supply_evaluator)
    # Add Time Window constraint
    time_evaluator = CreateTimeEvaluator(data).time_evaluator
    add_time_window_constraints(routing, data, time_evaluator)
    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    printer = ConsolePrinter(data, routing, assignment)
    printer.print()
    # for i in range(len(path_route)):
    #     print(path_route[i].getNode())
    l = [list()]*data.num_vehicles
    time_dict = {}
    c = 0
    for x in path_route:
        l[c] = []
        for y in x:
            if(y.getxy() != data.locations[data.depot]):
                l[c].append(y.getxy())
                time_dict[y.getxy()] = y.gettime()
        # print("Route : ",l[c])
        # print("Points not in present list : ")
        # print((list(set(data.locations)-set(l[c])-set([data.locations[data.depot]]))))
        c += 1
    sameTime = []
    locs = []
    for loclist in l:
        for loc in loclist:
            # print("For location : ", loc)
            loc_time = time_dict[loc]
            Otherpoints = []
            for key, value in time_dict.items():
                if(value == loc_time and key != loc):
                    Otherpoints.append(key)
            locs.append(loc)
            sameTime.append(nearest(loc, Otherpoints))
            # print(loc, sameTime)
    k = 0
    global output
    output = [list()] * data.num_vehicles
    temp_output=[]
    global cost
    vehicle_swap=0
    for vehicle_nbr in range(data.num_vehicles):
        l = []
        for i in range(1, len(path_route[vehicle_nbr]) - 1):
            if (sameTime[k] != None):
                l.append(cost_fn_distance(vehicle_nbr, sameTime, i, k))
                temp_output=swap_decider(vehicle_nbr, sameTime,k, cost_fn_distance(vehicle_nbr, sameTime, i, k), locs)
                if temp_output!=[]:
                    vehicle_swap = vehicle_nbr
                    output[vehicle_nbr]=temp_output
            k += 1
        # print(vehicle_nbr)
        cost.append(l)
    # print(cost)
    # print(route_dists)

    print("\nAfter Optimization:\n")
    if temp_output!=None:
        for i in range(data.num_vehicles):
            if i == vehicle_swap:
                # print(vehicle_swap)
                continue
            else:
                output[i] = []
                for j in range(len(path_route[i])):
                    # print(Vehiclei,loc_node_to_index(path_route[i][j].getxy()))
                    output[i].append(loc_node_to_index(path_route[i][j].getxy()))
            # for i in range(4):
    for i in range(data.num_vehicles):
        print("Vehicle {0}: {1}\n".format(i, output[i]))

        # print(neighbour_vehi)

    # for i in range(data.num_vehicles):
    #     swap_decider(i)
    #     print("  ")

    # for d in range(len(sameTime)):
    #     if sameTime[d] is not None:
    #         print(sameTime[d])
    # print(time_dict)

    # print(path_route[0][1].getNode())
    # print(path_route[0][-2].getNode())
    # print(pack[0].getpackagedetails())
    # node=(342,240)
    # vehicle_id= find_path(node)
    # for i in range(len(path_route[vehicle_id])):
    #     print(path_route[vehicle_id][i].getxy())


if __name__ == '__main__':
    main()
