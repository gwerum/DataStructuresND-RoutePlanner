from heapq import heappop, heappush
from math import sqrt
import pdb

class RoutePlanner():
    """docstring for RoutePlanner"""
    def __init__(self, M):
        self.cities = M.intersections
        self.neighbours = M.roads

    def compute_distance(self, city_A, city_B):
        pos_A, pos_B = self.cities[city_A], self.cities[city_B]
        dist_x, dist_y = pos_A[0]-pos_B[0], pos_A[1]-pos_B[1]
        return sqrt(dist_x**2 + dist_y**2)

    def compute_step_cost(self, current_city, next_city):
        g_path_cost = self.compute_distance(current_city, next_city)
        h_estimated_dist_to_target = self.compute_distance(current_city, self.target)
        f_step_cost = g_path_cost + h_estimated_dist_to_target
        return f_step_cost

    def compute_shortest_path(self, start_city, target_city):
        self.frontier = []
        self.explored = []
        self.target = None
        self.target = target_city
        current_city = start_city
        self.explored.append(current_city)
        current_cost = 0
        current_path = [start_city]
        heappush(self.frontier, (current_cost, current_path))

        while current_city != target_city:
            if target_city == 24:
                self.print_preliminary_results(current_city, current_path, current_cost)
            
            removed_current_city_from_frontier = heappop(self.frontier)

            for neighbour in self.neighbours[current_city]:
                if neighbour not in self.explored:
                    self.explored.append(neighbour)
                    step_cost = self.compute_step_cost(current_city, neighbour)
                    new_path_cost = current_cost + step_cost
                    new_path = current_path + [neighbour]
                    heappush(self.frontier, (new_path_cost, new_path))

            current_cost, current_path = self.frontier[0]
            current_city = current_path[-1]


            #pdb.set_trace()
        return current_cost, current_path

    def print_preliminary_results(self, current_city, current_path, current_cost):
        print("\nCurrent city: {}".format(current_city))
        print("\nCurrently explored: {}".format(self.explored))
        print("\nCurrent path: {}, Current cost: {}".format(current_path, current_cost))
        print("\nCurrent frontier: {}".format(self.frontier))

def shortest_path(M,start,goal):
    print("shortest path called")
    print("\nMap intersections:\n {}".format(M.intersections))
    print("\nMap roads:\n {}".format(M.roads))

    route_planner = RoutePlanner(M)

    print("\nCompute shortest path from {} to {}".format(start, goal))

    cost, shortest_path = route_planner.compute_shortest_path(start, goal)
    
    return shortest_path
