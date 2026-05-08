import heapq
import math
import random
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np


class Node:
    def __init__(self, x, y, g=float('inf'), rhs=float('inf'), parent=None):
        self.x = x
        self.y = y
        self.g = g # cost to reach this node
        self.rhs = rhs  # one-step lookahead cost
        self.neighbors = []
        self.parent = parent

def heuristic(node, goal):
    return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2)

def movement_cost(dx, dy):
    if dx != 0 and dy != 0:
        return math.sqrt(2)
    else:
        return 1


class DStarLite:

    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.queue = []  #priority queue
        self.k_m = 0  # cost increase
        self.path = []
        self.neighbors = []



        # Initialize start and goal nodes first
        self.start_node = Node(self.start[0], self.start[1])  # Note: assuming (x, y) format for consistency
        self.goal_node = Node(self.goal[0], self.goal[1])  # Note: assuming (x, y) format for consistency

        # Then initialize all nodes in the grid
        self.nodes = {(x, y): Node(x, y) for y in range(len(grid)) for x in range(len(grid[0]))}

        # Now set neighbors for each node, since start_node and goal_node are defined
        for node in self.nodes.values():
            node.neighbors = self.getNeighbors(node)
            print(f"Node ({node.x}, {node.y}) neighbors: {[(neighbor.x, neighbor.y) for neighbor in node.neighbors]}")
            if node == self.goal_node:
                print(f'Valid neighbor for goal node: {[(neighbor.x, neighbor.y) for neighbor in node.neighbors]}')

        self.initialize()

 
    def getNeighbors(self, node):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy
            if 0 <= nx < len(self.grid[0]) and 0 <= ny < len(self.grid):
                if self.grid[ny][nx] in [0, 2, 3]:
                    neighbors.append(self.nodes[(nx, ny)])
                    # print(f"Valid neighbor for node ({node.x}, {node.y}): ({nx}, {ny})")
                    if (node.x, node.y) == (self.goal_node.x, self.goal_node.y):
                        print(f'Valid neighbor for goal node {(node.x, node.y)}: ({nx}, {ny})')
                # else:
                #     print(f"Invalid (wall) neighbor for node ({node.x}, {node.y}): ({nx}, {ny})")
            # else:
            #     print(f"Out of bounds neighbor for node ({node.x}, {node.y}): ({nx}, {ny})")
        return neighbors



    def initialize(self):
        self.grid[self.goal[1]][self.goal[0]] = 3
        self.grid[self.start[1]][self.start[0]] = 2          
        # self.start_node = Node(self.start[1], self.start[0])
        # self.goal_node = Node(self.goal[1], self.goal[0])
        print(f'start and goal nodes {self.start_node.x, self.start_node.y}, {self.goal_node.x, self.goal_node.y}')
        self.goal_node.rhs = 0
        self.insertInQueue(self.goal_node)


    def insertInQueue(self, node):
        # remove if already in queue
        print(f"Inserting node ({node.x}, {node.y}) with key: {self.calculateKey(node)}")
        self.queue = [(k_old, n) for k_old, n in self.queue if n!= node]
        heapq.heappush(self.queue,(self.calculateKey(node), node))
        print(f"New RHS for ({node.x}, {node.y}): {node.rhs}")


    def calculateKey(self, node):
        return (min(node.g, node.rhs) + heuristic(node, self.start_node) + self.k_m, min(node.g, node.rhs))


    def updateVertex(self, node):
        print(f"Updating vertex ({node.x}, {node.y})")

        if node != self.goal_node:
            node.rhs = min(neighbor.g + movement_cost(abs(neighbor.x - node.x), abs(neighbor.y - node.y)) for neighbor in node.neighbors)
        if node in self.queue:
            self.queue = [(pri, n) for pri, n in self.queue if n != node]
            heapq.heapify(self.queue)
        if node.g != node.rhs:
            self.insertInQueue(node)


    def computeShortestPath(self):
        while self.queue and (self.queue[0][0] < self.calculateKey(self.start_node) or self.start_node.rhs != self.start_node.g):
            k_old, current_node = heapq.heappop(self.queue)

            current_node.neighbors = self.getNeighbors(current_node)  # Ensure neighbors are up-to-date
            # Proceed with processing

            print(f"Processing node ({current_node.x}, {current_node.y}) with old key: {k_old} and new key: {self.calculateKey(current_node)}")
            print(f"Current node neighbors before processing: {[(neighbor.x, neighbor.y) for neighbor in current_node.neighbors]}")
            if k_old < self.calculateKey(current_node):
                self.insertInQueue(current_node)
                print('k_old_self.caculateKey, insertInQueue')
            elif current_node.g > current_node.rhs:
                current_node.g = current_node.rhs
                print('find shorter path, current_node.g = current_node.rhs')
            else:
                current_node.g = float('inf')
                self.updateVertex(current_node)
                print('obstacle, update vertex')
            for neighbor in current_node.neighbors:
                self.updateVertex(neighbor)
        self.reconstructPath()


    def reconstructPath(self):
        print("Reconstructing path...")
        path = []
        current = self.goal_node

        while current != self.start_node:
            if not current.neighbors:
                print("No neighbors found, stopping path reconstruction.")
                break
            # Ensure we're moving towards the start by selecting the neighbor with a lower g value
            next_node = min(current.neighbors, key=lambda n: n.g)
            if next_node.g >= current.g:
                print("No progress towards start, stopping path reconstruction.")
                break
            path.append(next_node)
            current = next_node

        # while current != self.start_node:
        #     print(f"Adding node ({current.x}, {current.y}) to path")
        #     if not current.neighbors:  # Check if neighbors list is empty
        #         print("No neighbors found, stopping path reconstruction.")
        #         break  # Or handle differently
        #     current = min(current.neighbors, key=lambda n: n.g)
        #     path.append(current)
        path.append(self.start_node)  # add start position
        print("Path reconstruction complete.")
        return path[::-1]
    
    



############################################################################
########################## for testing D Star Lite #########################
    
def generate_maze_binary(size):
    """Generate a simple maze with a guaranteed path from start to goal."""
    maze = [[0 for _ in range(size)] for _ in range(size)]
    
    # Define start and goal positions
    start = (1, 0)
    goal = (size - 2, size - 1)
    
    # Create a path from start to goal
    x, y = start
    while x < goal[0]:
        maze[y][x] = 0
        x += 1
    while y < goal[1]:
        maze[y][x] = 0
        y += 1
    
    # Add random walls, avoiding start, goal, and the path
    for _ in range(size * 2):
        wall_x, wall_y = random.randint(1, size - 2), random.randint(1, size - 2)
        if (wall_x, wall_y) is not near (start, goal) and maze[wall_y][wall_x] == 0:
            maze[wall_y][wall_x] = 1

    # Mark the start and goal
    maze[start[1]][start[0]] = 2
    maze[goal[1]][goal[0]] = 3
    print(f'maze {maze}')
    return maze

def near(point_a, point_b, distance=3):
    """Check if point_a is within a certain distance from point_b."""
    return abs(point_a[0] - point_b[0]) <= distance and abs(point_a[1] - point_b[1]) <= distance

def print_maze(maze):
    symbols = {0: ' ', 1: '█', 2: 'S', 3: 'G'}  # Mapping of maze elements to symbols
    for row in maze:
        print(''.join([symbols[cell] for cell in row]))

def print_maze_matrix(maze):
    for row in maze:
        print(' '.join(str(cell) for cell in row))


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

def plot_maze(maze, path= None):
    fig, ax = plt.subplots()
    ax.imshow(maze, cmap=cm.get_cmap('Dark2'), interpolation='nearest')
    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, color="red", linewidth=2)

    start_x, start_y = np.where(maze == 2)
    goal_x, goal_y = np.where(maze == 3)
    ax.plot(start_x, start_y, marker="o", color="green", markersize=10)
    ax.plot(goal_x, goal_y, marker="o", color="blue", makersize=10)

    plt.show()    

def main():
    # Generate a new maze
    maze = generate_maze_binary(5)
    print_maze_matrix(maze)

    # Find start and end positions in the maze
    start, end = find_start_end_positions(maze)

    
    # Initialize the DStarLite algorithm with the maze
    d_star_lite = DStarLite(maze, start, end)
    d_star_lite.computeShortestPath()
    path = d_star_lite.reconstructPath()
    print("Path:", [(node.x, node.y) for node in path])

    
    # Print the path or some representation of it
    print(d_star_lite)

if __name__ == '__main__':
    main()