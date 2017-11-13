from PIL import Image
from scipy import misc
import numpy as np
from pandas import *
import time
import heapq

##Setting variable file_name to the maze image
file_name = '201x201.png'

##Opening the image as well as collecting the width and height of the maze
image = Image.open(file_name)
im_width = image.size[0]
im_height = image.size[1]

##Reading pixel data and drawing in walls on the array map
print('Reading image file... \n')
map = np.zeros([im_width, im_height], dtype=np.int)
im_data = misc.imread(file_name)

for x in range(im_width):
        for y in range(im_height):
            if 255 in im_data[x,y]:
                map[x,y] = 1

##Initiating the list walls
walls = set()

##Adding all walls to the list of walls
for x in range(im_width):
    for y in range(im_height):
        if map[x,y] == 0:
            checker = np.empty(shape=[1,2])
            checker = x,y
            walls.add(checker)

##Draws in the solution path
def draw_path(maze, id, style):
    r = '1'
    if 'path' in style and id in style['path']: r = '2'
    if id in maze.walls: r = '0'
    return r

##Stores the solved maze in array map
def draw_solution(maze, width=2, **style):
    for x in range(maze.width):
        for y in range(maze.height):
            map[x,y] = ((draw_path(maze, (x, y), style)))

##Checks cells for validity
class cell_checker:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

##Assigns weights to each cell within the maze
class maze_weights(cell_checker):
    def __init__(self, width, height):
        super().__init__(width, height)
        self.weights = {}
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)

maze = maze_weights(im_width, im_height)
maze.walls = walls

class priority_list:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

##A* pathfinding function
def a_star_search(maze, start, goal):
    maze_area = priority_list() ##Sets the variable frontier to the returned value of priority_list
    maze_area.put(start, 0)
    visited = {} ##Initiates the variable came_from
    cost_so_far = {} ##Initiates the variable cost_so_far
    visited[start] = None
    cost_so_far[start] = 0
    
    while not maze_area.empty(): ##While the variable frontier isn't empty
        current = maze_area.get() ##Sets current to the current cell
        
        if current == goal: ##Checks if the current cell is equal to the goal cell and stops search if it is
            break
        
        for next in maze.neighbors(current): ##For the next neighbour to the current cell
            new_cost = cost_so_far[current] + maze.cost(current, next) ##Initiates and sets the variable new_cost as the current cost added to the running cost
            if next not in cost_so_far or new_cost < cost_so_far[next]: ##If the next neighbour isn't in the running costs or the new cost is less than the new total cost
                cost_so_far[next] = new_cost ##Updates the cost so far
                priority = new_cost + heuristic(goal, next) ##Sets the priority for next move
                maze_area.put(next, priority) 
                visited[next] = current ##Adds the current cell to the visited cells
    
    return visited, cost_so_far

##Start timer for solution
start = time.time()

##Find the start and end locations by locating the white square in the top and bottom most rows
for x in range(im_width):
    if map[0,x] > 0:
        start_pos = 0,x

for x in range(im_width):
    if map[im_height -1 ,x] > 0:
        end_pos = im_height -1 ,x

##Use A* algorithm to pathfind through the maze
print('Finding the most optinal path through your maze... \n')
came_from, cost_so_far = a_star_search(maze, start_pos, end_pos)

##Draw the maze with the path drawn in
print('Path found, creating solution image...\n')
draw_solution(maze, path=reconstruct_path(came_from, start=start_pos, goal=end_pos))

##Exchanges solution symbols for correct RGB codes
solution_out = []
print('Exporting solution image... \n')
for x in range(im_width):
    for y in range(im_height):
        if map[x,y] == 0:
            solution_out.append((0,0,0))
        elif map[x,y] == 2:
            solution_out.append((255,0,0))
        else:
            solution_out.append((255,255,255))

##Creates solution image and saves to solution folder
image_out = Image.new('RGB', (im_width, im_height))
image_out.putdata(solution_out)
image_out.save('solution.png')

##End timer for solution
end = time.time()

##Print the time elapsed and the confirmation message
print('Solved! The solution has been saved in root folder: "solution.png"')
print('Time taken to find shortest path:', round(end - start,2), 'seconds')
