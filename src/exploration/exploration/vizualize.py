
from a_star import *
from explorer_algorithm import *

import pygame

# Initialize Pygame
pygame.init()

# Set up some constants
WIDTH, HEIGHT = 1000, 1000  # Window size
ROWS, COLS = 100, 100  # Grid size
CELL_SIZE = WIDTH // COLS  # Size of a cell

# Create the window
win = pygame.display.set_mode((WIDTH, HEIGHT))

obs_grid = generate_grid(ROWS)
viz_grid = generate_grid(ROWS, True)

obs_grid = add_obs(obs_grid, (0,0), (ROWS-1, COLS-1))
OBS_LIST = [(i, j) for i in range(ROWS) for j in range(COLS) if obs_grid[i][j] == 1]

# 
def draw_grid(rows, cols, width):
    for i in range(rows):
        pygame.draw.line(win, pygame.Color('white'), (0, i * width), (width, i * width))
        for j in range(cols):
            pygame.draw.line(win, pygame.Color('white'), (j * width, 0), (j * width, width))

def draw_walls(obs_list):
    for obs in obs_list:
        pygame.draw.rect(win, pygame.Color('black'), (obs[0]*CELL_SIZE, obs[1]*CELL_SIZE, CELL_SIZE, CELL_SIZE))

def draw_visible(visible):
    for i, point in enumerate(visible):
        pygame.draw.rect(win, pygame.Color('white'), (point[0]*CELL_SIZE, point[1]*CELL_SIZE, CELL_SIZE, CELL_SIZE))
    pygame.display.update()

def draw_path(point):
    # if point == path[0]:
    #     pygame.draw.rect(win, pygame.Color('blue'), (point[0] * CELL_SIZE, point[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
    # elif point == path[-1]:
    #     pygame.draw.rect(win, pygame.Color('red'), (point[0] * CELL_SIZE, point[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
    # else:
    pygame.draw.rect(win, pygame.Color('green'), (point[0] * CELL_SIZE, point[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
    draw_visible(cells)
    pygame.display.update()
    pygame.time.delay(100)
    

def main():
    run = True

    paths, cells = explore(obs_grid, viz_grid, (0,0))

    counter = 0

    while run:
        draw_grid(win, ROWS, COLS, WIDTH)
        draw_walls()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    draw_path(paths[counter], cells[counter])
                    counter += 1
                    
        
    pygame.quit()

if __name__ == "__main__":
    main()