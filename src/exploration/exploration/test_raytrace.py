from a_star import *
from explorer_algorithm import *

import pygame

pygame.init()


WIDTH, HEIGHT = 1000, 1000  # Window size
ROWS, COLS = 100, 100  # Grid size
CELL_SIZE = WIDTH // COLS  # Size of a cell

win = pygame.display.set_mode((WIDTH, HEIGHT))

def draw_walls(obs_list):
    print('drawing walls')
    for obs in obs_list:
        pygame.draw.rect(win, pygame.Color('orange'), (obs[0]*CELL_SIZE, obs[1]*CELL_SIZE, CELL_SIZE, CELL_SIZE))
    pygame.display.update()


def draw_grid(rows, cols, width):
    for i in range(rows):
        pygame.draw.line(win, pygame.Color('white'), (0, i * width), (width, i * width))
        for j in range(cols):
            pygame.draw.line(win, pygame.Color('white'), (j * width, 0), (j * width, width))
    pygame.display.update()


def draw_point(point):
    pygame.draw.rect(win, pygame.Color('green'), (point[0] * CELL_SIZE, point[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
    pygame.display.update()

def draw_visible(visible):
    print('drawing visible')
    for point in visible:
        pygame.draw.rect(win, pygame.Color('white'), (point[0]*CELL_SIZE, point[1]*CELL_SIZE, CELL_SIZE, CELL_SIZE))
    pygame.display.update()


def main():
    size = 100
    start = (50, 50, 0)
    end = (size-2, size-2)
    viz_grid = generate_grid(size, True)
    obs_grid = generate_grid(size)
    obs_grid = add_obs(obs_grid, start, end)

    obs_list = [(i, j) for i in range(size) for j in range(size) if obs_grid[i][j] == 1]

    path = []
    for angle in range(0, 360, 45):
        print(angle)
        pose = (start[0], start[1], angle)
        path.append(pose)


    tmp = Astar(obs_grid, start, end)
    path.extend(tmp)
    all_cells = []
    for point in path:
        tmp = seen_cells(obs_grid, viz_grid, (point[0], point[1], point[2]))
        all_cells.append(tmp)
    
    draw_grid(ROWS, COLS, WIDTH)

    run = True
    counter = 0
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    # Add a new point at a random location to the list of points
                    draw_visible(all_cells[counter])
                    draw_point(path[counter])
                    print(path[counter][2])
                    draw_walls(obs_list)
                    counter += 1

                    pygame.display.update()

    pygame.quit()


    
if __name__ == '__main__':
    main()