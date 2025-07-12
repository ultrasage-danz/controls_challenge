from . import BaseController

class Controller(BaseController):
    def __init__(self, kp=0.8, ki=0.04, kd=0.2, alpha=0.4):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_lat_accel = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_lat_accel = None

    def update(self, target_lataccel, current_lataccel, state, future_plan=None):
        # dt = state.get('dt', 0.05)  # Default timestep
        dt = getattr(state, 'dt', 0.05)  # Safely get dt attribute, fallback to 0.05 if missing
        
        error = target_lataccel - current_lataccel
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if self.prev_lat_accel is not None else 0.0

        pid_output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        control = self.alpha * target_lataccel + (1 - self.alpha) * pid_output

        self.prev_error = error
        self.prev_lat_accel = current_lataccel
        return control