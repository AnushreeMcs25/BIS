
import numpy as np

np.random.seed(42)

population_size = 20
generations = 30

dt = 0.01
time = np.arange(0, 5, dt)
target = 1.0

def fitness(pid):
    kp, ki, kd = pid

    speed = 0
    integral = 0
    prev_error = 0
    total_error = 0

    for t in time:
        error = target - speed

        integral += error * dt
        derivative = (error - prev_error) / dt

        control = kp * error + ki * integral + kd * derivative
        control = np.clip(control, 0, 1)

        speed += (control - speed) * dt / 0.5

        total_error += abs(error) * dt
        prev_error = error

    return total_error


population = np.random.uniform(
    [0, 0, 0],
    [5, 5, 1],
    (population_size, 3)
)

best_error = float('inf')
best_pid = None

for gen in range(generations):

    errors = np.array([fitness(p) for p in population])

    idx = np.argmin(errors)

    if errors[idx] < best_error:
        best_error = errors[idx]
        best_pid = population[idx].copy()

    print(f"Gen {gen:02d} | Best Error: {best_error:.5f} | Best [Kp, Ki, Kd]: {np.round(best_pid, 4)}")

    new_population = [best_pid.copy()]

    while len(new_population) < population_size:

        parents = population[np.argsort(errors)[:10]]

        p1, p2 = parents[np.random.choice(10, 2, replace=False)]

        child = (p1 + p2) / 2

        if np.random.rand() < 0.8:
            child += np.random.normal(0, 0.2, 3)

        child = np.clip(child, [0, 0, 0], [5, 5, 1])

        new_population.append(child)

    population = np.array(new_population)

print("...")
print("\nOptimization Complete.")
print("Optimal Controller Parameters (Kp, Ki, Kd):")
print(best_pid)



output:

Gen 00 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 01 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 02 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 03 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 04 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 05 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 06 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 07 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 08 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 09 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 10 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 11 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 12 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 13 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 14 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 15 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 16 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 17 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 18 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 19 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 20 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 21 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 22 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 23 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 24 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 25 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 26 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 27 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 28 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
Gen 29 | Best Error: 0.49998 | Best [Kp, Ki, Kd]: [2.2803 3.9259 0.1997]
...

Optimization Complete.
Optimal Controller Parameters (Kp, Ki, Kd):
[2.28034992 3.92587981 0.19967378]
