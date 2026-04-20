from dataclasses import dataclass, field


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    output_min: float
    output_max: float
    integral_limit: float
    integral: float = field(default=0.0, init=False)
    previous_error: float = field(default=0.0, init=False)
    initialized: bool = field(default=False, init=False)

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            derivative = 0.0
        else:
            derivative = 0.0 if not self.initialized else (error - self.previous_error) / dt
            self.integral += error * dt
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        self.initialized = True
        return max(self.output_min, min(output, self.output_max))
