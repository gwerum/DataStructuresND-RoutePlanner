from heapq import heappop, heappush, heapify
from math import sqrt
import pdb

class Frontier():
    """docstring for Frontier"""
    def __init__(self):
        self.frontier = []
        self.cities = set()

    def __repr__(self):
        return str(self.frontier)

    def push(self, total_cost, path_cost, path):
        self.cities.add(path[-1])
        heappush(self.frontier, (total_cost, path_cost, path))

    def pop(self, index = None):
        if not index:
            total_cost, path_cost, path = heappop(self.frontier)
        else:
            total_cost, path_cost, path = self.frontier[index]
            self.frontier[index] = self.frontier[-1]
            self.frontier.pop()
            heapify(self.frontier)
        popped_city = path[-1]
        self.cities.remove(popped_city)
        return popped_city, total_cost, path_cost, path

    def lookup(self, city):
        index = 0
        for total_cost, path_cost, path in self.frontier:
            if path[-1] == city:
                return index, total_cost, path_cost, path
            index += 1
        return None, None, None

    def exists(self, city):
        if city in self.cities:
            return True
        return False

class RoutePlanner():
    """docstring for RoutePlanner"""
    def __init__(self, M):
        self.cities = M.intersections
        self.neighbours = M.roads

    def compute_distance(self, city_A, city_B):
        pos_A, pos_B = self.cities[city_A], self.cities[city_B]
        dist_x, dist_y = pos_A[0]-pos_B[0], pos_A[1]-pos_B[1]
        return sqrt(dist_x**2 + dist_y**2)

    def estimate_step_cost(self, current_city, next_city):
        step_cost = self.compute_distance(current_city, next_city)
        h_residual_cost = self.compute_distance(next_city, self.target)
        return step_cost, h_residual_cost

    def compute_shortest_path(self, start_city, target_city):
        self.loop_count = 0
        # Initialize target, explore list and frontier list
        self.target = target_city
        self.frontier = Frontier()
        self.explored = [start_city]
        # Initialize loop variables
        current_city, current_path_cost, current_path = start_city, 0, [start_city]

        while current_city != self.target:
            self.print_preliminary_results(current_city, current_path, current_path_cost)

            for neighbour_city in self.neighbours[current_city]:
                if neighbour_city not in self.explored:
                    # Estimate total cost
                    step_cost, residual_cost = self.estimate_step_cost(current_city, neighbour_city)
                    path_cost = current_path_cost + step_cost
                    total_cost = path_cost + residual_cost # f = g + h
                    neighbour_path = current_path + [neighbour_city]
                    # Check if neighbour already exists in frontier list
                    if self.frontier.exists(neighbour_city):
                        f_index, f_total_cost, f_path_cost, f_path = self.frontier.lookup(neighbour_city)
                        if (path_cost < f_path_cost):
                            self.frontier.pop(f_index)
                            self.frontier.push(total_cost, path_cost, neighbour_path)
                    else:
                        self.frontier.push(total_cost, path_cost, neighbour_path)
            
            current_city, current_total_cost, current_path_cost, current_path = self.frontier.pop()
            self.explored.append(current_city)

        return current_path_cost, current_path

    def print_preliminary_results(self, current_city, current_path, current_cost):
        print("\n########### Loop {} ########### ".format(self.loop_count))
        print("\nCurrent city: {}".format(current_city))
        print("\nCurrent path: {}, Current cost: {}".format(current_path, current_cost))
        print("\nExplored cities: {}".format(self.explored))
        print("\nFrontier cities: {}".format(self.frontier.cities))
        print("\nCurrent frontier: {}".format(self.frontier))
        self.loop_count += 1

def shortest_path(M,start,goal):
    print("shortest path called")
    print("\nMap intersections:\n {}".format(M.intersections))
    print("\nMap roads:\n {}".format(M.roads))

    route_planner = RoutePlanner(M)

    print("\nCompute shortest path from {} to {}".format(start, goal))

    cost, shortest_path = route_planner.compute_shortest_path(start, goal)
    
    return shortest_path
