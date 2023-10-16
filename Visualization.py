import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import csv
import os

def readData(id,filename):
    solution_1 = []
    solution_2 = []
    solution_3 = []
    solution_4 = []
    solution_5 = []
    solution_6 = []
    with open(filename, 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        
        for row in csv_reader:
            if row['id'] == str(id):
                solution_1.append(tuple(map(int,row['transporter_1'].strip('()').split(','))))
                solution_2.append(tuple(map(int,row['transporter_2'].strip('()').split(','))))
                solution_3.append(tuple(map(int,row['transporter_3'].strip('()').split(','))))
                solution_4.append(tuple(map(int,row['transporter_4'].strip('()').split(','))))
                solution_5.append(tuple(map(int,row['transporter_5'].strip('()').split(','))))
                solution_6.append(tuple(map(int,row['transporter_6'].strip('()').split(','))))

    return solution_1, solution_2, solution_3, solution_4, solution_5, solution_6

# Update function for the animation
def update(frame):
    ax.clear()

    # Re-plot the grid
    for r in range(rows):
        for c in range(cols):
            ax.add_patch(plt.Rectangle((c, rows - 1 - r), 1, 1, color='white'))
    #rows - 1 - r
    for idx, agent_solution in enumerate(solutions):
        if frame < len(agent_solution):
            from_square, to_square, agent_frame, agent_id = agent_solution[frame]
            from_row, from_col = divmod(from_square, cols)
            to_row, to_col = divmod(to_square, cols)
            # Update the scatter plot data
            '''
            scatters[idx] = ax.scatter([from_col + frame * (to_col - from_col) / 1],
                                       [rows - 1 - from_row - frame * (from_row - to_row) / 1],
                                       color=agent_colors[agent_id % len(agent_colors)])
            '''
            scatters[idx] = ax.scatter([from_col + (to_col - from_col) / 1],
                                       [from_row + (to_row - from_row) / 1],
                                       color=agent_colors[agent_id % len(agent_colors)])
    # Show grid lines
    ax.grid(True)
     # Use annotate to display time step
    ax.annotate(f'Time Step: {frame}', xy=(0.5, -0.1), xycoords='axes fraction', ha='center', color='black')


# Define the grid dimensions
rows = 14
cols = 4

# Create a color map for agents
agent_colors = ['b', 'g', 'r', 'c', 'm', 'y']

# Load your multi-agent pathfinding solution here
# Format: [(from_square, to_square, frame, id), ...]
#filePath = os.path.join('data', 'LPsolution.csv')
filePath = 'E:\ThesisProject\FastLab\data\LPsolution.csv'
solution_1, solution_2, solution_3, solution_4, solution_5, solution_6 = readData(1,filePath)
solutions = [solution_1, solution_2, solution_3, solution_4, solution_5, solution_6]
  
# Create the figure and axis
fig, ax = plt.subplots()

# Create the grid
grid = np.zeros((rows-1, cols-1))

# Plot the initial grid and empty scatter plots
for r in range(rows):
    for c in range(cols):
        ax.add_patch(plt.Rectangle((c, rows - 1 - r), 1, 1, color='lightgrey'))
scatters = [ax.scatter([], [], color=agent_colors[agent_id % len(agent_colors)]) for agent_id, _, _, _ in solutions[0]]

# Set axis limits and labels
ax.set_xlim(0, cols-1)
ax.set_ylim(0, rows-1)
ax.set_xticks([])
ax.set_yticks([])

plt.title("Multi-Agent Pathfinding Solution Animation")

# Create the animation
max_frames = max(len(agent_solution) for agent_solution in solutions)
ani = animation.FuncAnimation(fig, update, frames=max_frames, repeat=False, interval=500)

plt.show()