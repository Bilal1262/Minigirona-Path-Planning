import numpy as np
import math


def wrap_angle(angle):
    """
    Wraps an angle to the range [-π, π].

    Args:
        angle (float): The angle to wrap.

    Returns:
        float: The wrapped angle.
    """
    return (angle + (2.0 * np.pi * np.floor((np.pi - angle) / (2.0 * np.pi))))


class StateValidityChecker:
    """
    Checks if a position or a path is valid given an occupancy map.
    """

    def __init__(self, distance=0.2, is_unknown_valid=True):
        """
        Constructor for the StateValidityChecker class.

        Args:
            distance (float): Radius around the robot used to check occupancy.
            is_unknown_valid (bool): If True, unknown space is considered valid.
        """
        self.map = None  # 2D array of integers representing the occupancy map
        self.resolution = None  # Map resolution (size of a cell)
        self.origin = None  # World position of cell (0, 0) in the map
        self.there_is_map = False  # Flag to indicate if the map has been set
        self.distance = distance  # Radius for collision checking
        self.is_unknown_valid = is_unknown_valid  # Flag to treat unknown space as valid

    def set(self, data, resolution, origin):
        """
        Sets the occupancy map, its resolution, and origin.

        Args:
            data (np.array): 2D occupancy map.
            resolution (float): Map resolution.
            origin (list): World position of the map's origin.
        """
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True

    def is_valid(self, pose):
        """
        Checks if a pose is valid (not in collision).

        Args:
            pose (list or np.array): The pose to check.

        Returns:
            bool: True if the pose is valid, False otherwise.
        """
        # Convert the position from world to map coordinates
        grid_pos = self._position_to_map_(pose[:2])

        # Check if the position is out of bounds
        if not (0 <= grid_pos[0] < self.map.shape[0] and 0 <= grid_pos[1] < self.map.shape[1]):
            return self.is_unknown_valid

        # Check occupancy within a circular radius
        radius_cells = int(np.ceil(self.distance / self.resolution))
        radius_sq = (self.distance / self.resolution) ** 2

        unknown_found = False
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                # Skip if outside the circular radius
                if dx**2 + dy**2 > radius_sq:
                    continue

                # Check cell validity
                check_x = grid_pos[0] + dx
                check_y = grid_pos[1] + dy
                if 0 <= check_x < self.map.shape[0] and 0 <= check_y < self.map.shape[1]:
                    cell_value = self.map[check_x, check_y]
                    if cell_value == 100:  # Occupied cell
                        return False
                    elif cell_value == -1:  # Unknown cell
                        unknown_found = True
                else:
                    # Out-of-bounds cells are considered unknown
                    unknown_found = True

        return self.is_unknown_valid if unknown_found else True

    def check_path(self, path):
        """
        Checks if a path is collision-free.

        Args:
            path (list): List of waypoints in the path.

        Returns:
            bool: True if the path is valid, False otherwise.
        """
        if not path:
            return False

        stepsize = self.distance / 2

        for i in range(len(path) - 1):
            start = np.array(path[i])
            goal = np.array(path[i + 1])
            difference = goal - start
            dist = np.linalg.norm(difference)
            steps = max(int(np.ceil(dist / stepsize)), 1)

            for step in range(steps + 1):
                point = start + (difference * step / steps)
                if not self.is_valid(point):
                    return False

        return True

    def _position_to_map_(self, p):
        """
        Converts a world position to map coordinates.

        Args:
            p (list or np.array): World position [x, y].

        Returns:
            np.array: Map coordinates [x, y].
        """
        if self.origin is None or self.map is None:
            return np.array([-1, -1])

        p = np.array(p)
        cell_p = np.round((p - self.origin) / self.resolution).astype(int)
        return cell_p


class Planner:
    """
    Planner class for path planning using RRT.
    """

    def __init__(self, state_validity_checker, dominion, max_iterations=10000, step_size=0.5, goal_bias=1):
        """
        Constructor for the Planner class.

        Args:
            state_validity_checker (StateValidityChecker): Object for collision checking.
            dominion (list): Dominion [min_x, max_x, min_y, max_y] for sampling.
            max_iterations (int): Maximum number of iterations for RRT.
            step_size (float): Step size for local planning.
            goal_bias (float): Probability of sampling the goal.
        """
        self.svc = state_validity_checker
        self.dominion = dominion
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.sampling = 5

    def rand_conf(self, q_goal):
        """
        Samples a random configuration.

        Args:
            q_goal (list or np.array): Goal position.

        Returns:
            tuple: A valid random configuration.
        """
        for _ in range(self.sampling):
            if np.random.rand() < self.goal_bias:
                rand_point = tuple(q_goal)
            else:
                rand_point = np.array([
                    np.random.uniform(self.dominion[0], self.dominion[1]),
                    np.random.uniform(self.dominion[2], self.dominion[3])
                ])
            if self.svc.is_valid(rand_point):
                return rand_point

        return tuple(q_goal)

    def euclidean(self, p1, p2):
        """
        Computes the Euclidean distance between two points.

        Args:
            p1 (list or np.array): First point.
            p2 (list or np.array): Second point.

        Returns:
            float: Euclidean distance.
        """
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    def local_planner(self, qrand, qnear, actual):
        """
        Local planner to move from qnear towards qrand.

        Args:
            qrand (list or np.array): Random point.
            qnear (list or np.array): Nearest point.
            actual (float): Step size.

        Returns:
            tuple: New point after moving towards qrand.
        """
        max_step = self.step_size
        min_step = 0.1
        distance = self.euclidean(qnear, qrand)

        if self.svc.check_path([qnear, qrand]):
            actual = min(max_step, distance)
        else:
            actual = min_step

        if distance == 0:
            return None

        # Calculate direction vector
        dx = qrand[0] - qnear[0]
        dy = qrand[1] - qnear[1]
        norm = math.sqrt(dx ** 2 + dy ** 2)
        if norm > 0:
            dx /= norm
            dy /= norm

        # Create new node position
        new_x = qnear[0] + dx * actual
        new_y = qnear[1] + dy * actual

        return (new_x, new_y)

    def is_collision_free(self, p1, p2):
        """
        Checks if the path between two points is collision-free.

        Args:
            p1 (list or np.array): Start point.
            p2 (list or np.array): End point.

        Returns:
            bool: True if the path is collision-free, False otherwise.
        """
        p1 = np.array(p1)
        p2 = np.array(p2)
        steps = 200  # Number of intermediate points to check
        for t in np.linspace(0, 1, steps):
            point = p1 + t * (p2 - p1)
            if not self.svc.is_valid(point):
                return False

        return True

    def near_points(self, nodes, new_node, radius):
        """
        Finds nodes within a specified radius of a new node.

        Args:
            nodes (list): List of nodes.
            new_node (tuple): New node.
            radius (float): Search radius.

        Returns:
            list: Nodes within the radius.
        """
        near = []
        for node in nodes:
            dist = self.euclidean(node, new_node)
            if dist <= radius:
                near.append(node)
        return near

    def reconstruct_path(self, parents, start, goal):
        """
        Reconstructs the path from start to goal.

        Args:
            parents (dict): Parent-child relationships.
            start (tuple): Start node.
            goal (tuple): Goal node.

        Returns:
            list: Reconstructed path.
        """
        path = [np.array(goal)]
        current = tuple(goal)
        start = tuple(start)
        while current != start:
            current = parents[current]
            path.append(np.array(current))
        return path[::-1]

    def reconstruct_smooth_path(self, parents, start, goal):
        """
        Reconstructs and smooths the path from start to goal.

        Args:
            parents (dict): Parent-child relationships.
            start (tuple): Start node.
            goal (tuple): Goal node.

        Returns:
            list: Smoothed path.
        """
        path = [np.array(goal)]
        current = tuple(goal)
        start = tuple(start)
        while current != start:
            current = parents[current]
            path.append(current)
        path.reverse()

        # Smooth the path
        smoothed_path = path[:]
        n = len(smoothed_path) - 1
        while n > 0:
            i = 0
            while i < n:
                if self.is_collision_free(smoothed_path[i], smoothed_path[n]):
                    smoothed_path = smoothed_path[:i + 1] + smoothed_path[n:]
                    n = i
                    break
                i += 1
            else:
                n -= 1

        return smoothed_path

    def path_cost(self, parents, node):
        """
        Computes the cost of the path from start to a node.

        Args:
            parents (dict): Parent-child relationships.
            node (tuple): Target node.

        Returns:
            float: Path cost.
        """
        total_cost = 0
        while parents[node] is not None:
            node1 = parents[node]
            cost = self.euclidean(node, node1)
            total_cost += cost
            node = parents[node]
        return total_cost

    def compute_path(self, q_start, q_goal):
        """
        Computes a path from start to goal using RRT.

        Args:
            q_start (list or np.array): Start position.
            q_goal (list or np.array): Goal position.

        Returns:
            list: Path from start to goal.
        """
        self.q_start = tuple(q_start)
        self.q_goal = tuple(q_goal)

        nodes = [self.q_start]
        parents = {self.q_start: None}
        edges = []
        path = []
        iter = None

        for i in range(self.max_iterations):
            # Sample a random point
            rand_point = self.rand_conf(self.q_goal)

            # Find the nearest node
            nearest_node = min(nodes, key=lambda node: self.euclidean(node, rand_point))

            # Take a step towards the random point
            new_node = self.local_planner(rand_point, nearest_node, self.step_size)
            if new_node is None:
                continue

            new_node = tuple(new_node)

            # Check if the new node is collision-free
            if self.is_collision_free(nearest_node, new_node):
                nodes.append(new_node)
                parents[new_node] = nearest_node
                edges.append((nearest_node, new_node))

                # Check if the goal is reached
                if self.euclidean(new_node, self.q_goal) < 0.5:
                    print(f"Goal reached in {i} iterations!")
                    iter = i
                    path = self.reconstruct_smooth_path(parents, self.q_start, new_node)
                    path.append(np.array(self.q_goal))
                    return path

        iter = i
        print("Goal not reached within maximum iterations.")
        return None


class DWAPlanner:
    """
    Dynamic Window Approach (DWA) planner for local motion planning.
    """

    def __init__(self, max_lin_vel=0.15, max_ang_vel=0.3, max_lin_acc=0.5, max_ang_acc=0.3, dt=0.1,
                 predict_time=1.0, vel_resolution=0.02, goal_weight=3.5, obstacle_weight=1.0,
                 velocity_weight=0.2, robot_radius=0.15):
        """
        Constructor for the DWAPlanner class.

        Args:
            max_lin_vel (float): Maximum linear velocity.
            max_ang_vel (float): Maximum angular velocity.
            max_lin_acc (float): Maximum linear acceleration.
            max_ang_acc (float): Maximum angular acceleration.
            dt (float): Time step for simulation.
            predict_time (float): Prediction time horizon.
            vel_resolution (float): Velocity resolution for sampling.
            goal_weight (float): Weight for goal alignment.
            obstacle_weight (float): Weight for obstacle avoidance.
            velocity_weight (float): Weight for velocity.
            robot_radius (float): Radius of the robot for collision checking.
        """
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        self.max_lin_acc = max_lin_acc
        self.max_ang_acc = max_ang_acc
        self.dt = dt
        self.predict_time = predict_time
        self.vel_resolution = vel_resolution
        self.goal_weight = goal_weight
        self.obstacle_weight = obstacle_weight
        self.velocity_weight = velocity_weight
        self.robot_radius = robot_radius

        # State variables
        self.current_v = 0.0
        self.current_w = 0.0

    def compute_velocity_command(self, current_pose, goal_point, svc):
        """
        Computes the best velocity command using DWA.

        Args:
            current_pose (list or np.array): Current robot pose [x, y, theta].
            goal_point (list or np.array): Goal position [x, y].
            svc (StateValidityChecker): State validity checker.

        Returns:
            tuple: Best linear and angular velocities (v, w).
        """
        dw = self.calculate_dynamic_window()
        best_score = -float('inf')
        best_v, best_w = 0.0, 0.0
        self.all_trajectories = []
        self.best_trajectory = None

        for v in np.linspace(dw[0], dw[1], int((dw[1] - dw[0]) / self.vel_resolution) + 1):
            for w in np.linspace(dw[2], dw[3], int((dw[3] - dw[2]) / self.vel_resolution) + 1):
                trajectory = self.simulate_trajectory(current_pose, v, w, svc)
                if not trajectory:
                    continue

                self.all_trajectories.append(trajectory)
                goal_score = self.calculate_goal_score(trajectory[-1], goal_point)
                obstacle_score = self.calculate_obstacle_score(trajectory, svc)
                velocity_score = self.calculate_velocity_score(v)

                total_score = (self.goal_weight * goal_score +
                               self.obstacle_weight * obstacle_score +
                               self.velocity_weight * velocity_score)

                if total_score > best_score:
                    best_score, best_v, best_w = total_score, v, w
                    self.best_trajectory = trajectory

        return best_v, best_w

    def calculate_dynamic_window(self):
        """
        Calculates the dynamic window for velocity sampling.

        Returns:
            list: Dynamic window [v_min, v_max, w_min, w_max].
        """
        return [
            max(-self.max_lin_vel, self.current_v - self.max_lin_acc * self.dt),
            min(self.max_lin_vel, self.current_v + self.max_lin_acc * self.dt),
            max(-self.max_ang_vel, self.current_w - self.max_ang_acc * self.dt),
            min(self.max_ang_vel, self.current_w + self.max_ang_acc * self.dt)
        ]

    def simulate_trajectory(self, current_pose, v, w, svc):
        """
        Simulates a trajectory given a velocity command.

        Args:
            current_pose (list or np.array): Current robot pose [x, y, theta].
            v (float): Linear velocity.
            w (float): Angular velocity.
            svc (StateValidityChecker): State validity checker.

        Returns:
            list: Simulated trajectory.
        """
        trajectory = []
        pose = np.copy(current_pose)

        for _ in range(int(self.predict_time / self.dt)):
            pose = self.motion_model(pose, v, w)
            if not svc.is_valid(pose[:2]):
                return []
            trajectory.append(pose)

        return trajectory

    def motion_model(self, pose, v, w):
        """
        Unicycle motion model.

        Args:
            pose (list or np.array): Current pose [x, y, theta].
            v (float): Linear velocity.
            w (float): Angular velocity.

        Returns:
            np.array: New pose after motion.
        """
        x, y, theta = pose
        new_theta = theta + w * self.dt
        new_x = x + v * np.cos(new_theta) * self.dt
        new_y = y + v * np.sin(new_theta) * self.dt
        return np.array([new_x, new_y, new_theta])

    def calculate_goal_score(self, final_pose, goal_point):
        """
        Computes the goal alignment score.

        Args:
            final_pose (list or np.array): Final pose in the trajectory.
            goal_point (list or np.array): Goal position.

        Returns:
            float: Goal alignment score.
        """
        dx = goal_point[0] - final_pose[0]
        dy = goal_point[1] - final_pose[1]
        return 1.0 / (math.hypot(dx, dy) + 1e-5)

    def calculate_obstacle_score(self, trajectory, svc):
        """
        Computes the obstacle avoidance score.

        Args:
            trajectory (list): Simulated trajectory.
            svc (StateValidityChecker): State validity checker.

        Returns:
            float: Obstacle avoidance score.
        """
        min_dist = float('inf')
        for pose in trajectory:
            dist = self.calculate_obstacle_distance(pose[:2], svc)
            min_dist = min(min_dist, dist)
            if dist < self.robot_radius:
                return -float('inf')
        return min(min_dist, 2.0)

    def calculate_obstacle_distance(self, point, svc):
        """
        Calculates the distance to the nearest obstacle.

        Args:
            point (list or np.array): Point to check.
            svc (StateValidityChecker): State validity checker.

        Returns:
            float: Distance to the nearest obstacle.
        """
        max_search_radius = 2.0  # meters
        resolution = svc.resolution
        grid_radius = int(max_search_radius / resolution)

        grid_pos = svc._position_to_map_(point)
        if not (0 <= grid_pos[0] < svc.map.shape[0] and 0 <= grid_pos[1] < svc.map.shape[1]):
            return 0.0

        # Expand search until obstacle found
        for r in range(0, grid_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx**2 + dy**2 > r**2:
                        continue
                    x = grid_pos[0] + dx
                    y = grid_pos[1] + dy
                    if 0 <= x < svc.map.shape[0] and 0 <= y < svc.map.shape[1]:
                        if svc.map[x, y] == 100:
                            return math.hypot(dx, dy) * resolution
        return max_search_radius

    def calculate_velocity_score(self, v):
        """
        Computes the velocity score.

        Args:
            v (float): Linear velocity.

        Returns:
            float: Velocity score.
        """
        return abs(v) / self.max_lin_vel


def compute_path(start_p, goal_p, state_validity_checker, dominion, max_iterations=2000):
    """
    Computes a path from start to goal using RRT.

    Args:
        start_p (list or np.array): Start position.
        goal_p (list or np.array): Goal position.
        state_validity_checker (StateValidityChecker): State validity checker.
        dominion (list): Dominion [min_x, max_x, min_y, max_y] for sampling.
        max_iterations (int): Maximum number of iterations for RRT.

    Returns:
        list: Path from start to goal.
    """
    planner = Planner(state_validity_checker, dominion, max_iterations=max_iterations, step_size=0.3, goal_bias=0.2)
    path = planner.compute_path(start_p, goal_p)
    return path if path is not None else []


def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    """
    Computes the velocity command to move from current position to goal.

    Args:
        current (list or np.array): Current pose [x, y, theta].
        goal (list or np.array): Goal position [x, y].
        Kv (float): Proportional gain for linear velocity.
        Kw (float): Proportional gain for angular velocity.

    Returns:
        tuple: Linear and angular velocities (v, w).
    """
    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    heading_to_goal = math.atan2(dy, dx)
    angle = heading_to_goal - current[2]
    angle_wrap = wrap_angle(angle)
    distance = math.hypot(dx, dy)

    v = Kv * distance
    w = Kw * angle_wrap

    return v, w