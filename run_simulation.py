import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from dynamics import bicycle_model
from mpcc import MPCC

# Initialize controller
controller = MPCC(horizon=25, dt=0.1)

# Corrected Initial state: Pointing along the track (pi/2) and starting at max speed (5.0)
state = np.array([
    4.0,       # x
    0.0,       # y
    np.pi / 2, # psi
    3.0,       # v
    0.0        # theta
])

# --- Define Physical Dimensions ---
agent_L = 0.4
agent_W = 0.2
agent_radius = np.hypot(agent_L / 2, agent_W / 2) 

static_box_w = 2.4
static_box_h = 1.6
static_obs_radius = np.hypot(static_box_w / 2, static_box_h / 2)

dyn_box_w = 2.0
dyn_box_h = 2.0
dyn_obs_radius = np.hypot(dyn_box_w / 2, dyn_box_h / 2)

# Calculate the Configuration Space (Combined) Radii for the solver
static_combined_radius = static_obs_radius + agent_radius
dyn_combined_radius = dyn_obs_radius + agent_radius

# --- Define Obstacle States ---
# Static Obstacle data [cx, cy, total_radius]
static_obs_data = np.array([0.0, 5.0, static_combined_radius])

# Dynamic Obstacle starting state
dyn_x = 4.0
dyn_y = -4.0
dyn_vx = -1.5 
dyn_vy = 0.5  

# Tracking arrays for plotting
trajectory_x = []
trajectory_y = []
dyn_history_x = []
dyn_history_y = []

print("Starting Simulation...")

for t in range(150):

    # 1. Predict dynamic obstacle trajectory for the solver's horizon
    dyn_obs_traj = np.zeros((3, 25))
    for k in range(25):
        dyn_obs_traj[0, k] = dyn_x + dyn_vx * (k * 0.1)
        dyn_obs_traj[1, k] = dyn_y + dyn_vy * (k * 0.1)
        dyn_obs_traj[2, k] = dyn_combined_radius # Pass the expanded radius!

    # 2. Pass to solver
    try:
        u, prediction = controller.solve(state, static_obs_data, dyn_obs_traj)
    except Exception as e:
        print(f"Solver failed at step {t} due to infeasibility (Likely unavoidable collision).")
        break

    a = u[0]
    delta = u[1]
    v_theta = u[2]

    # 3. Step Dynamics
    next_state = bicycle_model(state[:4], [a, delta], dt=0.1)
    theta_next = state[4] + 0.1 * v_theta
    
    state = np.array([next_state[0], next_state[1], next_state[2], next_state[3], theta_next])

    # 4. Save trajectories and move dynamic obstacle
    trajectory_x.append(state[0])
    trajectory_y.append(state[1])
    
    dyn_history_x.append(dyn_x)
    dyn_history_y.append(dyn_y)
    
    dyn_x += dyn_vx * 0.1
    dyn_y += dyn_vy * 0.1

    print(f"Step {t}: x={state[0]:.2f}, y={state[1]:.2f}")


# --- Plotting Visualization ---
track_theta = np.linspace(0, 2*np.pi, 200)
track_x = 5 * np.cos(track_theta)
track_y = 5 * np.sin(track_theta)

plt.figure(figsize=(10, 10))
ax = plt.gca()

# Plot Track and Vehicle Path
plt.plot(track_x, track_y, '--', label='Track')
plt.plot(trajectory_x, trajectory_y, '-r', linewidth=2, label='Vehicle Center Path')

# Plot Physical Static Box
static_rect = patches.Rectangle(
    (static_obs_data[0] - static_box_w/2, static_obs_data[1] - static_box_h/2), 
    static_box_w, static_box_h, 
    linewidth=2, edgecolor='black', facecolor='gray', label='Physical Static Box'
)
ax.add_patch(static_rect)

# Plot Solver Keep-Out Zone (Minkowski Sum)
static_circle = patches.Circle(
    (static_obs_data[0], static_obs_data[1]), static_combined_radius,
    edgecolor='red', facecolor='none', linestyle='--', label='Solver Keep-Out Zone'
)
ax.add_patch(static_circle)

# Plot Dynamic Obstacle Path
plt.plot(dyn_history_x, dyn_history_y, ':', color='orange', linewidth=3, label='Dynamic Box Path')

plt.legend()
plt.axis('equal')
plt.title("MPCC with Configuration Space Obstacles")
plt.show()