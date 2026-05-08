from hmac import new
import random
from tkinter.tix import MAX
from tracemalloc import start
from flask import g
from nbformat import convert
import numpy as np
import math

from scipy import rand
from sympy import N
from a_star import *
import numpy as np

"""
pusedo code:
grid_viz
grid_obs
while grid_viz.cleared() < 0.8:
    while counter < 10:
        pick random point
        calc path to point
        calc info gain of path
        if info gain is high enough, move to point
        else, repeat, check if new better than old, if so save, else repeat counter++
    pick best path
    move along path
    if obstacle found in path, stop and recalulate best path

"""


###CONSTANTS###  
N_SIZE = 100
FOV = 45
MAX_RANGE = 5
###############

def seen_cells(obs_grid, viz_grid, pose):
    """
    Get the cells that are visible from the agent's current position in a cone infront of it.
    if the cell is an obstacle (1), break, if the cell is free and known (0), continue. Unknown is (9)
    """
    visible_cells = []
    obs_list = []
    for angle in range(-FOV//2, FOV//2):
        # Adjust the angle based on the direction the agent is facing
        adjusted_angle = pose[2] + angle
        for r in range(MAX_RANGE):
            dx = pose[0] + r * math.cos(math.radians(adjusted_angle))
            dy = pose[1] + r * math.sin(math.radians(adjusted_angle))
            x = pose[0] + int(dx)
            y = pose[1] + int(dy)
            line = bresenham_line((pose[0], pose[1]), (x, y))
            for point in line:
                if obs_grid[point[0]][point[1]] == 1:
                    obs_list.append(point)
                    break
                elif viz_grid[point[0]][point[1]] == 0:
                    continue
                elif viz_grid[point[0]][point[1]] == -1:
                    visible_cells.append(point)
    return visible_cells, obs_list


def bresenham_line(start, end):
    """Generate points along a line using Bresenham's line algorithm."""
    points = []
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    x, y = x1, y1
    if abs(dy) < abs(dx):
        # Slope < 1
        if dx < 0:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            dx, dy = -dx, -dy
            x, y = x1, y1

        p = 2*dy - dx
        while x <= x2:
            points.append((x, y))
            x += 1
            if p < 0:
                p += 2*dy
            else:
                y += 1 if y1 < y2 else -1
                p += 2*(dy - dx)
    else:
        # Slope >= 1
        if dy < 0:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            dx, dy = -dx, -dy
            x, y = x1, y1

        p = 2*dx - dy
        while y <= y2:
            points.append((x, y))
            y += 1
            if p < 0:
                p += 2*dx
            else:
                x += 1 if x1 < x2 else -1
                p += 2*(dx - dy)

    return points


def random_point(grid):
    """Gets a random point on the grid that is not an obstacle."""
    x = random.randint(0, N_SIZE-1)
    y = random.randint(0, N_SIZE-1)
    if grid[y][x] != 0:
        return random_point(grid)
    return (x, y)

def amount_cleared(grid):
    """Calculates the amount of the grid that has been cleared."""
    # print('Calculating amount cleared...')
    cleared = 0
    for i in range(N_SIZE):
        for j in range(N_SIZE):
            if grid[i][j] == 0:
                cleared += 1
    print(f'Amount cleared: {cleared/N_SIZE**2}')
    return cleared/N_SIZE**2   

def convert_seen(grid, cells):
    """Converts the cells that have been seen to 0 in the visibility grid."""
    print('Converting seen cells...')

    for cell_set in cells:
        for cell in cell_set:
            print(cell[0])
            print(cell[1])
            grid[cell[0]][cell[1]] = 0
    return grid 

def calc_info_gain(obs_grid, viz_grid, path):
    """
    Calculate the information gain of a path.
    Information gain is the number of new cells that will be seen along the path.
    Seen cells is a list of sets containing all the cells that will be seen from each point in the path.
    """
    info_gain = 0
    cells = []
    for point in path:
            x = point[0]
            y = point[1]
            direction = point[2]
            new_cells = seen_cells(obs_grid, viz_grid, (x, y, direction))
            info_gain += len(new_cells)
            cells.append(new_cells)
        
    return info_gain, cells

def start_seq(obs_grid, viz_grid, pose):
    """Start the exploration sequence with turning in place"""

    for angle in range(0, 360, 45):
        pose[2] = angle
        seen_cells = seen_cells(obs_grid, viz_grid, pose)
        convert_seen(viz_grid, seen_cells)


def explore(obs_grid, viz_grid, pose, simulation=False):
    """Explore the grid until a given amount of it has been cleared or have run n number of iterations.
    pusedo code:
    grid_viz
    grid_obs
    while grid_viz.cleared() < 0.8:
        while counter < 10:
            pick random point
            calc path to point
            calc info gain of path
            if info gain is high enough, move to point
            else, repeat, check if new better than old, if so save, else repeat counter++
        pick best path
        move along path
        if obstacle found in path, stop and recalulate best path
    """
    print('Started exploring...')
    path_taken = []
    cells_seen = []

    if simulation:
        tmp = []
        for angle in range(0, 360, 45):
            pose = (pose[0], pose[1], angle)
            visible_cells = seen_cells(obs_grid, viz_grid, pose)
            tmp.append(visible_cells)
            convert_seen(viz_grid, tmp)
       
        cells_seen.append(tmp)
        path_taken.append(pose)
            

    for _ in range(3):
        print('Exploring...')
        print(f'Amount cleared: {amount_cleared(viz_grid)}')
        counter = 0
        best_path = None
        cells = None
        gain = 0
        while counter < 3:
            print(f'Finding best path iteration...{counter}')

            goal = random_point(obs_grid)
            print(f'Random point: {goal}')

            path = Astar(obs_grid, pose, goal)
            print('Path found')

            tmp_gain, tmp_cells = calc_info_gain(obs_grid, viz_grid, path)
            print(f'Info gain: {tmp_gain}')

            if tmp_gain > gain:
                best_path = path
                cells = tmp_cells
                gain = tmp_gain
                print('New best path found')
            
            counter += 1
        
        convert_seen(viz_grid, cells)
        print('Moving along path...')
        pose = best_path[-1]
        path_taken.extend(best_path)
        cells_seen.extend(cells)

    print('Done exploring')

    return path_taken, cells_seen
                



def generate_grid(size, visualize=False):
    """Generates a simple binary maze of given size using '1' for walls and '0' for unknown space."""
    if visualize:
        cell = -1
    else:
        cell = 0
    maze = [[cell for _ in range(size)] for _ in range(size)]

    print('generate maze')
    # Add borders
    for i in range(size):
        maze[0][i] = maze[size-1][i] = 1  # Top and bottom
        maze[i][0] = maze[i][size-1] = 1  # Left and right

    return maze
    
def add_obs(grid, start, end):
    # Add random walls inside, ensuring start and end are open
    for _ in range(N_SIZE*2):  # Just a simple heuristic for number of obstacles
        x, y = random.randint(1, N_SIZE-2), random.randint(1, N_SIZE-2)
        grid[y][x] = 1
    return grid
        

def main():
    """"
    viz_grid: grid that tracks what the robot has seen. 
    None = unkown 
    0 = free space
    1 = obstacle
    """
    viz_grid = generate_grid(100, visualize=True) # this is the visibility grid which tracks what the robot has seen?
    obs_grid = generate_grid(100) # this is the occpuyancy grid from the lidar
    start = (0,0)
    obs_grid = add_obs(obs_grid, start, (N_SIZE-1, N_SIZE-1))
    cleared = explore(obs_grid, viz_grid, start)


if __name__ == "__main__":
    main()