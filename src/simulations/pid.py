class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain

        self.integral = 0
        self.previous_error = 0

    def update(self, setpoint, current_value, dt):
        """
        Update the PID loop with the current value.

        setpoint: The desired value.
        current_value: The current value.
        dt: Time interval in seconds.
        returns: Control variable.
        """
        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update previous error
        self.previous_error = error

        return output
