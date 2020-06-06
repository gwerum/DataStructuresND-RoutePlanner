from helpers import load_map

from student_code import RoutePlanner

MAP_40_ANSWERS = [
    (5, 34, [5, 16, 37, 12, 34]),
    (5, 5,  [5]),
    (8, 24, [8, 14, 16, 37, 12, 17, 10, 24])
]

def test():
    map_40 = load_map('map-40.pickle')
    route_planner = RoutePlanner(map_40)
    correct = 0
    for start, goal, answer_path in MAP_40_ANSWERS:
        cost, path = route_planner.compute_shortest_path(start, goal)
        if path == answer_path:
            print("\nShortest path from {} to {} is:\n{}".format(start, goal, path))
            correct += 1
        else:
            print("For start:", start, 
                  "Goal:     ", goal,
                  "Your path:", path,
                  "Correct:  ", answer_path)
    if correct == len(MAP_40_ANSWERS):
        print("All tests pass! Congratulations!")
    else:
        print("You passed", correct, "/", len(MAP_40_ANSWERS), "test cases")


if __name__ == "__main__":
    test()
    