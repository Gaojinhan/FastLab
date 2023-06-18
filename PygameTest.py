import pygame
from pygame.locals import *

 

# Define constants for the window size and grid dimensions
GRID_WIDTH = 4
GRID_HEIGHT = 30

 

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

 

# Define agent positions
agent_positions = [
    (1, 2),  # Agent 1 start position (row, column)
    (1, 1),  # Agent 2 start position (row, column)
    # Add more agent positions here as needed
]

 

# Define agent goals
agent_goals = [
    (28, 3),  # Agent 1 goal position (row, column)
    (15, 2),  # Agent 2 goal position (row, column)
    # Add more agent goals here as needed
]

 
# Define the agent schedule
agent_schedule = [
    [[0, 1], [0, 2]],  # Agent 1 schedule: move right, move right
    [[1, 0], [1, 0]]   # Agent 2 schedule: move down, move down
    # Add more agent schedules here as needed
]

# Calculate the size of each grid block
BLOCK_WIDTH = 30
BLOCK_HEIGHT = 30

WIDTH = GRID_WIDTH * BLOCK_WIDTH
HEIGHT = GRID_HEIGHT * BLOCK_HEIGHT

# Initialize Pygame
pygame.init()

 

# Create the game window
window = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Multi-Agent Pathfinding Visualization")

 

# Set up the game clock
clock = pygame.time.Clock()

 

# Define the main game loop
def game_loop():
    running = True
    time_step = 0
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

 

        # Clear the window
        window.fill(WHITE)

 

        # Draw grid
        for row in range(GRID_HEIGHT):
            for col in range(GRID_WIDTH):
                pygame.draw.rect(window, BLACK, (col * BLOCK_WIDTH, row * BLOCK_HEIGHT, BLOCK_WIDTH, BLOCK_HEIGHT), 1)

 
        '''
        # Update agent positions based on the schedule
        for i, position in enumerate(agent_positions):
            row, col = position
            schedule = agent_schedule[i]
            if time_step < len(schedule):
                move = schedule[time_step]
                row += move[0]
                col += move[1]
                agent_positions[i] = [row, col]
        '''
 

        # Draw agents and goals
        for position, goal in zip(agent_positions, agent_goals):
            agent_x = position[1] * BLOCK_WIDTH + BLOCK_WIDTH // 2
            agent_y = position[0] * BLOCK_HEIGHT + BLOCK_HEIGHT // 2
            goal_x = goal[1] * BLOCK_WIDTH + BLOCK_WIDTH // 2
            goal_y = goal[0] * BLOCK_HEIGHT + BLOCK_HEIGHT // 2
            pygame.draw.circle(window, RED, (agent_x, agent_y), 10)  # Draw agent as a red circle
            pygame.draw.circle(window, GREEN, (goal_x, goal_y), 10)  # Draw goal as a green circle

 

        # Update the display
        pygame.display.flip()

 

        # Control the game's frame rate
        clock.tick(60)

 

        # Increment the time step
        time_step += 1

 

    # Quit the game
    pygame.quit()

 

# Start the game loop
game_loop()