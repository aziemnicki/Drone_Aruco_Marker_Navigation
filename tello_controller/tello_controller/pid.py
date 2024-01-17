import numpy as np


class PIDController:
    def __init__(self, Kp, Ki, Kd, rotation=False):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = 0
        self.prev_error = 0
        self.integral = 0
        self.dt = 0.1
        self.rotation = rotation

    def __call__(self, current_value):
        error = self.setpoint - current_value

        if self.rotation:
            if error > np.pi:
                error -= 2 * np.pi
            elif error < -np.pi:
                error += 2 * np.pi

        self.integral += error

        # Proportional term
        P = self.Kp * error

        # Integral term
        I = self.Ki * self.integral

        # Derivative term
        D = self.Kd * (error - self.prev_error) / self.dt

        # Update previous error for the next iteration
        self.prev_error = error

        # Calculate the control signal
        output = P + I + D

        return output

    def reset_state(self):
        self.integral = 0
        self.prev_error = 0
