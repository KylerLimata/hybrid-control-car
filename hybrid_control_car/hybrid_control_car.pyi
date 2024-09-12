from typing import List


class CarSimulation:
    """
    An instance of a simulated robot car and its environment.
    """
    def reset_car(self) -> None:
        """
        Resets the simulated car to its initial state and position.
        """
    def create_floor(self) -> None:
        """
        Creates a large floor for the vehicle to drive around on. Useful for gathering data about the motion of the car.
        """
    def step(self, engine_force: float, steering_angle: float) -> List[float]:
        """
        Steps the simulation forward.

        :param engine_force: the force exerted by the car's engine.
        :param steering_angle: the steering angle of the front axis.
        :return: The current state of the car. The first two numbers are the horizontal translation 
        and the second two are the horizontal linear velocity
        """