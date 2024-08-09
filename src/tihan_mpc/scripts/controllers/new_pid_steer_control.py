class PIDController:
    def __init__(self, Kp, Ki, Kd, output_limits=(-134, 134)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits
        self.integral = 0
        self.previous_error = 0
        self.feedforward_gain = 134 / 370
    
    def update(self, setpoint, feedback):
        # Normalize angles from [-370, 370] to [-1, 1]
        setpoint_normalized = setpoint / 370
        feedback_normalized = feedback / 370
        
        # Calculate error
        error = setpoint_normalized - feedback_normalized
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative
        
        # PID output
        output = P + I + D
        
        # Feedforward term to directly drive to the setpoint
        feedforward = self.feedforward_gain * setpoint
        
        # Combined output
        output_combined = output + feedforward
        
        # Save error for next derivative calculation
        self.previous_error = error
        
        # Apply output limits
        output_combined = max(self.output_limits[0], min(self.output_limits[1], output_combined))
        
        return output_combined
    
pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
desired_angle = 100  # Desired steering angle in degrees
feedback_angle = 100  # Current feedback angle in degrees

steer_input = pid.update(desired_angle, feedback_angle)
print(f"Steer Input: {steer_input}")