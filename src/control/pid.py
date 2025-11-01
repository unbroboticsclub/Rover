# src/control/pid.py
class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0

    def update(self, value, dt):
        error = self.setpoint - value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*derivative
