import math
import heapq
import random

class Node:
    def __init__(self, x, y, g=0.0, h=0.0, parent=None):
        self.x = x 
        self.y = y
        self.g = g
        self.h = h 
        self.f = g + h
        self.parent = parent
        self.direction = 0

    def __lt__(self, other):
        return self.f < other.f
    def __hash__(self):
        return hash((self.x, self.y))
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y 


def heuristic(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

def new_waypoints(node):
    new_waypoints = [(0,1),(1,0), (0,-1),(-1,0),(1,1), (1, -1), (-1, -1), (-1, 1)]
    return [(dx, dy, movement_cost(dx, dy)) for dx, dy in new_waypoints]

def is_safe(node, grid, goal_value=3 ):
    rows, cols = len(grid), len(grid[0])
    safe = 0 <= node.x < rows and 0 <= node.y < cols and (grid[node.x][node.y] == 0 or grid[node.x][node.y] == goal_value)
    # if not safe:
    #     print(f"Node ({node.x}, {node.y}) is not safe or not traversable.")
    return safe

def reconstruct_path(current):
    path = []
    while current is not None:
        path.append((current.x, current.y, current.direction))
        current = current.parent
    print('arrive goal')
    return path[::-1]

def movement_cost(dx, dy):
    if dx != 0 and dy != 0:
        return math.sqrt(2)
    else:
        return 1

def Astar(grid, start, goal):
    """
    Direction: 
    0 degrees means right, 
    90 degrees means up,
    180 degrees means left, 
    270 degrees means down.
    """
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    grid[goal[1]][goal[0]] = 3
    grid[start[1]][start[0]] = 2

    open_list = []
    visited = set()
    heapq.heappush(open_list, start_node)
    cost_so_far = {start_node: 0.0}

    while open_list:
        current = heapq.heappop(open_list)
        if current in visited:
            continue
        visited.add(current)
        # print(f'visited add current: {current.x, current.y}')
        
        if current == goal_node:
            return reconstruct_path(current)
        
        for dx, dy, movement_cost in new_waypoints(current):
            next = Node(current.x + dx, current.y +dy)
            next.direction = math.degrees(math.atan2(dy, dx))
            if not is_safe(next, grid):
                continue
            new_cost = cost_so_far[current]+movement_cost
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                next.g = new_cost
                next.h = heuristic(next, goal_node)
                next.f = next.g + next.h
                next.parent = current
                heapq.heappush(open_list, next)
                #print(f'Exploring {next.x, next.y} with new cost {new_cost}.****************')
            else:
                #print(f'Higher cost path or exist node found to {next.x, next.y}, skipping.......')
                pass
    print('cannot find a valid path')
    return []



################ for testing Astar
def generate_maze_binary(size):
    """Generates a simple binary maze of given size using '1' for walls and '0' for paths."""
    maze = [[0 for _ in range(size)] for _ in range(size)]
    print('generate maze')
    # Add borders
    for i in range(size):
        maze[0][i] = maze[size-1][i] = 1  # Top and bottom
        maze[i][0] = maze[i][size-1] = 1  # Left and right
    
    # Add random walls inside, ensuring start and end are open
    for _ in range(size*2):  # Just a simple heuristic for number of obstacles
        x, y = random.randint(1, size-2), random.randint(1, size-2)
        maze[y][x] = 1
    
    # Ensure start (S) and end (E) are open and marked differently if needed
    maze[1][0] = 2  # Start, using '2' to distinguish, but you can treat it as '0' for path
    maze[size-2][size-1] = 3  # End, using '3' to distinguish, but you can treat it as '0' for path
    # Print the binary maze
    for row in maze:
        print(' '.join(str(cell) for cell in row))
    return maze


def find_start_end_positions(maze):
    """Find start (2) and end (3) positions in the maze."""
    start_pos = None
    end_pos = None
    for y, row in enumerate(maze):
        for x, value in enumerate(row):
            if value == 2:  # Start position
                start_pos = (y, x)
                print(f'start from {start_pos}')
            elif value == 3:  # End position
                end_pos = (y, x)
                print(f'end at {end_pos}')
    return start_pos, end_pos

def main():
    # Generate a new maze
    maze = generate_maze_binary(1000)
    
    # Find start and end positions in the maze
    start, end = find_start_end_positions(maze)

    
    # Initialize the AStar algorithm with the maze
    # a_star = Astar(maze, start,end)
    
    
    # # Print the path or some representation of it
    # print(a_star)


if __name__ == '__main__':
    main()

