
from helpers import Map, load_map, show_map
from student_code import shortest_path

#map_10 = load_map('map-10.pickle')
#show_map(map_10)

# map_40 is a bigger map than map_10
map_40 = load_map('map-40.pickle')
#show_map(map_40)

# run this code, note the effect of including the optional
# parameters in the function call.
#show_map(map_40, start=5, goal=34, path=[5,16,37,12,34])

######## Code ###########

start, goal = 5, 3

path = shortest_path(map_40, start, goal)

print("\nShortest path from {} to {} is:\n{}".format(start, goal, path))

if path == [5, 16, 37, 12, 34]:
    print("\ngreat! Your code works for these inputs!")
else:
    print("\nsomething is off, your code produced the following:")
    print(path)

######## Test cases ##########
'''
from test import test

test(shortest_path)
'''


