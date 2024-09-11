from hybrid_control_car import CarSimulation
import numpy as np

sim = CarSimulation()

np.set_printoptions(precision=3)

for i in range(60):
    state = sim.step(100.0, 0.0)
    state = np.array(state)

    print(f"{state}")