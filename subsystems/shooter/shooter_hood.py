import commands2
import math
import commands2


class ShooterHood(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self._gravity = 9.81

        self._hub_height = 1.8288
        self._shooter_height = 0.3048
        self._ball_weight = 0.215

        self._min_angle = math.radians(10)
        self._max_angle = math.radians(60)

        self._effciency_factor = 0.9

        self.with_RPS(60)

    def with_RPS(self, RPS):
        self._exit_velocity = RPS * (2 * math.pi) * self._effciency_factor
        return self

    def calculate_hood_angle(self, distance: float) -> float:
        delta_h = self._hub_height - self._shooter_height
        v = self._exit_velocity
        d = distance
        g = self._gravity

        discriminant = v**4 - g * (g * d**2 + 2 * delta_h * v**2)

        if discriminant < 0:
            return self._max_angle

        sqrt_term = math.sqrt(discriminant)

        theta = math.atan((v**2 - sqrt_term) / (g * d))

        return max(self._min_angle, min(self._max_angle, theta))

    def angle_hood(self, distance: float):
        target_angle = self.calculate_hood_angle(distance)
        self.set_angle(target_angle)

    def set_angle(self, angle):
        print("Set Hood Angle to", angle)
