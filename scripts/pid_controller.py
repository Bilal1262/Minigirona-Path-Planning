import numpy as np

class PIDController:
    def __init__(self):
        # PID parameters for each dimension
        self.kp = {'x': 0.3, 'y': 0.3, 'z': 0.5, 'theta': 0.1}
        self.ki = {'x': 0.1, 'y': 0.1, 'z': 0.1, 'theta': 0.1}
        self.kd = {'x': 0.1, 'y': 0.1, 'z': 0.1, 'theta': 0.1}
        
        # Initialize error terms
        self.prev_error = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'theta': 0.0}
        self.integral = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'theta': 0.0}
    
    def compute_velocities(self, current_pose, waypoint):
        """
        Compute velocities using PID control
        
        Args:
            current_pose (dict): Current pose with keys 'x', 'y', 'z', 'theta'
            waypoint (dict): Target waypoint with keys 'x', 'y', 'z', 'theta'
            
        Returns:
            dict: Velocity commands with keys 'linear' and 'angular'
                linear: dict with keys 'x', 'y', 'z'
                angular: dict with keys 'x', 'y', 'z'
        """
        # Calculate errors
        error = {
            'x': waypoint['x'] - current_pose['x'],
            'y': waypoint['y'] - current_pose['y'],
            'z': waypoint['z'] - current_pose['z'],
            'theta': waypoint['theta'] - current_pose['theta']
        }
        
        # Wrap theta error to [-pi, pi]
        error['theta'] = np.arctan2(np.sin(error['theta']), np.cos(error['theta']))
        
        # Calculate control signals
        control = {}
        for dim in ['x', 'y', 'z', 'theta']:
            # Update integral term
            self.integral[dim] += error[dim]
            
            # Calculate derivative term
            derivative = error[dim] - self.prev_error[dim]
            
            # Compute PID control
            control[dim] = (self.kp[dim] * error[dim] +
                          self.ki[dim] * self.integral[dim] +
                          self.kd[dim] * derivative)
            
            # Update previous error
            self.prev_error[dim] = error[dim]
        
        # Return velocity commands
        return {
            'linear': {
                'x': control['x'],
                'y': control['y'],
                'z': control['z']
            },
            'angular': {
                'x': 0.0,
                'y': 0.0,
                'z': control['theta']
            }
        }
    
    def reset(self):
        """Reset the PID controller's error terms"""
        self.prev_error = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'theta': 0.0}
        self.integral = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'theta': 0.0}
