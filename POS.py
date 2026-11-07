import numpy as np
import matplotlib.pyplot as plt

START = np.array([0.0, 0.0])
END = np.array([10.0, 10.0])

OBSTACLES = [
    {"center": np.array([3.5, 3.0]), "radius": 1.4},
    {"center": np.array([7.0, 6.0]), "radius": 1.4}
]

NUM_WAYPOINTS = 4
DIM = NUM_WAYPOINTS * 2

def check_segment_collision(p1, p2, center, radius):
    d = p2 - p1
    f = p1 - center
    a = np.dot(d, d)
    safety_margin = 0.2
    adjusted_radius = radius + safety_margin
    if a < 1e-9:
        return np.linalg.norm(p1 - center) < adjusted_radius
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - adjusted_radius * adjusted_radius
    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False
    discriminant = np.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    if (t1 >= 0 and t1 <= 1) or (t2 >= 0 and t2 <= 1) or (t1 < 0 and t2 > 1):
        return True
    return False

def calculate_fitness(particle_position):
    waypoints = particle_position.reshape((NUM_WAYPOINTS, 2))
    full_path = np.vstack([START, waypoints, END])
    total_distance = 0.0
    penalty = 0.0
    for i in range(len(full_path) - 1):
        p1 = full_path[i]
        p2 = full_path[i+1]
        total_distance += np.linalg.norm(p2 - p1)
        for obs in OBSTACLES:
            if check_segment_collision(p1, p2, obs["center"], obs["radius"]):
                penalty += 1e7
    return total_distance + penalty

NUM_PARTICLES = 350
MAX_ITERATIONS = 350
W_START = 0.9
W_END = 0.4
C1 = 1.8
C2 = 1.8

positions = np.random.uniform(0, 10, (NUM_PARTICLES, DIM))
velocities = np.random.uniform(-1.0, 1.0, (NUM_PARTICLES, DIM))
pbest_positions = np.copy(positions)
pbest_scores = np.array([calculate_fitness(p) for p in positions])

gbest_index = np.argmin(pbest_scores)
gbest_position = np.copy(pbest_positions[gbest_index])
gbest_score = pbest_scores[gbest_index]

for iteration in range(MAX_ITERATIONS):
    W = W_START - (W_START - W_END) * (iteration / MAX_ITERATIONS)
    for i in range(NUM_PARTICLES):
        r1, r2 = np.random.rand(DIM), np.random.rand(DIM)
        velocities[i] = (W * velocities[i] + 
                         C1 * r1 * (pbest_positions[i] - positions[i]) + 
                         C2 * r2 * (gbest_position - positions[i]))
        velocities[i] = np.clip(velocities[i], -1.0, 1.0)
        positions[i] = np.clip(positions[i] + velocities[i], 0, 10)
        score = calculate_fitness(positions[i])
        if score < pbest_scores[i]:
            pbest_scores[i] = score
            pbest_positions[i] = np.copy(positions[i])
            if score < gbest_score:
                gbest_score = score
                gbest_position = np.copy(positions[i])

sorted_indices = np.argsort(pbest_scores)
valid_paths = []
threshold_distance = 2.0

for idx in sorted_indices:
    score = pbest_scores[idx]
    if score >= 1e7:
        continue
    candidate_waypoints = pbest_positions[idx].reshape((NUM_WAYPOINTS, 2))
    candidate_path = np.vstack([START, candidate_waypoints, END])
    
    is_distinct = True
    for accepted_path in valid_paths:
        distance_diff = np.mean(np.linalg.norm(candidate_path[1:-1] - accepted_path[1:-1], axis=1))
        if distance_diff < threshold_distance:
            is_distinct = False
            break
    if is_distinct:
        valid_paths.append(candidate_path)
        if len(valid_paths) >= 2:
            break

plt.figure(figsize=(9, 9))
for idx, obs in enumerate(OBSTACLES):
    label = 'Obstacles' if idx == 0 else ""
    obstacle_circle = plt.Circle(obs["center"], obs["radius"], color='red', alpha=0.35, label=label)
    plt.gca().add_patch(obstacle_circle)

plt.scatter([START[0], END[0]], [START[1], END[1]], color='blue', s=150, zorder=10, label='Start/End')

colors = ['#1f77b4', '#ff7f0e']
for i, path in enumerate(valid_paths):
    plt.plot(path[:, 0], path[:, 1], '-o', color=colors[i % len(colors)],
             linewidth=2.5, markersize=6, alpha=0.8, label=f'Path {i+1}')

plt.xlim(-1, 11)
plt.ylim(-1, 11)
plt.grid(True, linestyle='--', alpha=0.5)
plt.title("Collision-Free Multiple Path Generation via PSO")
plt.legend(loc='upper left')
plt.show()



output:
