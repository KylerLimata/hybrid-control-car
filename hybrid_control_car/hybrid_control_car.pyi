from typing import List, Tuple

def simulate(x0, u, params):
    """
    Simulates the motion of a car

    :param x0: The initial state of the car.
    :param u: The control input.
    :param params: A dictionary of other simulation parameters.
    """


class CarSimulation:
    """
    An instance of a simulated robot car and its environment.
    """
    def reset_car(self, rotation: float) -> None:
        """
        Resets the simulated car to its initial state and position.

        :param rotation: the initial rotation of the car body.
        """
    def create_floor(self) -> None:
        """
        Creates a large floor for the vehicle to drive around on. Useful for gathering data about the motion of the car.
        """
    def step(self, timestep: int, engine_force: float, steering_angle: float) -> Tuple[List[float], bool, bool, bool]:
        """
        Steps the simulation forward and returns information about the simulation.
        Of the four return values, the last three only apply if you've constructed a race course.

        :param timestep: the timestep to simulate; if the car has already been simulated at this timestep, it will return
        the state from its stored history.
        :param engine_force: the force exerted by the car's engine.
        :param steering_angle: the steering angle of the front axis.
        :return: tuple (state, colliding, checkpoint, finish) 
            WHERE
            state Is the horizontal position, forward velocity scalar, and components of the horizontal forwards vector
            colliding Is whether the car is currently colliding with another object
            checkpoint Is whether the car has reached a checkpoint
            finish Is whether the car has reached the finish line
        """

class SimulationConfig:
    """
    Config for a simulation environment
    """

class SimulationEnvironment:
    """
    Simulation environment
    """

    def __init__(self, config: SimulationConfig): ...

    def step(x0, u): ...