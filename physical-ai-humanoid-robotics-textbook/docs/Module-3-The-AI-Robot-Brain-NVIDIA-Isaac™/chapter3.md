# Chapter 3: Path Planning for Bipedal Humanoid Movement

### 3.1 Introduction to Nav2 (Navigation 2)

Nav2 (Navigation 2) is the ROS 2 navigation framework that provides:

- **Path Planning**: Calculating collision-free routes through environments
- **Behavior Trees**: Flexible robot behavior coordination
- **Recovery Behaviors**: Handling navigation failures gracefully
- **Cost Maps**: Spatial representations of obstacles and traversability

### 3.2 Bipedal Navigation Challenges

Humanoid robots present unique navigation challenges compared to wheeled robots:

#### 3.2.1 Kinematic Constraints

```
Wheeled Robot:
- Instantaneous rotation possible
- Omnidirectional movement (some types)

Bipedal Humanoid:
- Forward walking is primary mode
- Lateral stepping more energy-expensive
- Rotation requires multi-step sequences
- Balance constraints limit acceleration
```

#### 3.2.2 Dynamic Balance Requirement

Bipedal walking requires continuous balance maintenance:

- **Zero Moment Point (ZMP)**: The point where vertical reaction force acts
- **Stability Region**: Area where ZMP must remain for stability
- **Center of Mass Trajectory**: Pre-planned to maintain stability

```python
# Example: Zero Moment Point Calculation
import numpy as np

def calculate_zmp(center_of_mass, velocity, acceleration, mass, g=9.81):
    """
    Calculate Zero Moment Point for bipedal stability
    
    Args:
        center_of_mass: [x, y, z] CoM position in meters
        velocity: [vx, vy, vz] CoM velocity in m/s
        acceleration: [ax, ay, az] CoM acceleration in m/s²
        mass: Robot mass in kg
    
    Returns:
        zmp: [x, y] ZMP position in horizontal plane
    """
    g = 9.81
    z_com = center_of_mass[2]
    
    # ZMP calculation using inverted pendulum model
    zmp_x = center_of_mass[0] - (velocity[2] * velocity[0]) / g - (acceleration[0] * z_com) / g
    zmp_y = center_of_mass[1] - (velocity[2] * velocity[1]) / g - (acceleration[1] * z_com) / g
    
    return np.array([zmp_x, zmp_y])
```

### 3.3 Nav2 Architecture for Humanoids

#### 3.3.1 Costmap and Environment Representation

Nav2 uses costmaps to represent robot's understanding of traversable space:

**Static Costmap:**
- Pre-loaded maps from SLAM or survey data
- Represents known static obstacles
- High confidence but may become stale

**Dynamic Costmap:**
- Updated from sensor data in real-time
- Detects dynamic obstacles (humans, moving objects)
- Combined with static map for comprehensive view

```python
# Example: Nav2 Costmap Configuration
costmap_config = {
    'global_frame': 'map',
    'robot_base_frame': 'base_link',
    'update_frequency': 10.0,      # Hz
    'publish_frequency': 5.0,       # Hz
    'width': 100,                   # cells
    'height': 100,
    'resolution': 0.05,             # meters per cell
    'inflation_radius': 0.5,        # meters
    'layers': {
        'static': {'map_topic': '/map'},
        'obstacles': {'sensor_topic': '/lidar/scan'},
        'inflation': {'cost_scaling_factor': 10.0}
    }
}
```

#### 3.3.2 Planners for Bipedal Motion

Nav2 supports multiple planning algorithms optimized for different scenarios:

**Theta* (Any-angle Path Planning):**
- Produces natural paths without grid artifacts
- Suitable for bipedal step placement
- Time-optimal trajectory generation

**Hybrid-A* (Kinodynamic Planning):**
- Respects vehicle dynamics and constraints
- Generates feasible bipedal walking sequences
- Handles non-holonomic constraints

```python
# Example: Configuring Theta* Planner
theta_star_params = {
    'inflation_cost_scaling_factor': 3.0,
    'cost_travel_multiplier': 2.0,
    'minimum_path_length': 0.1,
    'max_planning_time': 5.0,          # seconds
    'angle_quantization_divisions': 72  # 5-degree resolution
}

# For bipedal robots, prefer:
planner_type = 'nav2_theta_star_planner::ThetaStarPlanner'
```

### 3.4 Gait Adaptation and Step Planning

#### 3.4.1 Gait Types for Humanoid Navigation

```
Walking Gait (0.3-1.5 m/s):
- Natural bipedal walking motion
- Energy-efficient for sustained movement
- Good stability margin

Running Gait (1.5-3.0 m/s):
- Dynamic phases with flight
- Higher energy consumption
- Reduced stability margin

Turning and Spinning:
- Multi-step rotation sequences
- Coordinated hip and foot placement
- Smooth curvature following
```

#### 3.4.2 Step Footprint Generation

Steps must be placed on collision-free, stable surfaces:

```python
# Example: Footstep Planning for Humanoids
class FootstepPlanner:
    def __init__(self, max_step_length=0.5, max_turn_angle=30):
        self.max_step_length = max_step_length
        self.max_turn_angle = max_turn_angle
        
    def plan_footsteps(self, start_pose, goal_pose, costmap):
        """
        Generate sequence of footsteps to reach goal
        
        Args:
            start_pose: [x, y, theta] initial humanoid position
            goal_pose: [x, y, theta] desired position
            costmap: 2D occupancy grid
        
        Returns:
            footsteps: List of [x, y, theta] for each step
        """
        footsteps = []
        current_pose = start_pose
        
        while not self._reached_goal(current_pose, goal_pose):
            # Generate candidate next steps
            candidates = self._generate_step_candidates(current_pose, goal_pose)
            
            # Select best step (collision-free, stable surface)
            next_step = self._select_best_step(candidates, costmap)
            
            footsteps.append(next_step)
            current_pose = next_step
        
        return footsteps
    
    def _generate_step_candidates(self, current_pose, goal_pose, num_candidates=8):
        """Generate candidate footsteps at different angles"""
        candidates = []
        for angle in np.linspace(0, 2*np.pi, num_candidates):
            x = current_pose[0] + self.max_step_length * np.cos(angle)
            y = current_pose[1] + self.max_step_length * np.sin(angle)
            candidates.append([x, y, angle])
        return candidates
    
    def _select_best_step(self, candidates, costmap):
        """Select step with best cost (closeness to goal + stability)"""
        best_candidate = None
        best_cost = float('inf')
        
        for candidate in candidates:
            cost = self._evaluate_step_cost(candidate, costmap)
            if cost < best_cost:
                best_cost = cost
                best_candidate = candidate
        
        return best_candidate
    
    def _evaluate_step_cost(self, step, costmap):
        """Evaluate step based on collision and stability"""
        # Check if foot placement is collision-free
        if not self._is_collision_free(step, costmap):
            return float('inf')
        
        # Evaluate stability (surface normal, friction, etc.)
        stability_score = self._calculate_stability(step)
        
        return stability_score
```

#### 3.4.3 Navigation Feedback and Control Loop

```python
# Nav2 Controller Loop for Bipedal Robots
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

rclpy.init()
nav = BasicNavigator()

# Wait for nav to be ready
nav.waitUntilNav2Active()

# Set goal
goal_pose = [10.0, 5.0, 0.0]  # [x, y, theta]
nav.goToPose(goal_pose)

# Monitor navigation
while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    
    # Adapt gait if necessary based on feedback
    if feedback.navigation_time > 30.0:  # Timeout
        nav.cancelTask()
        break
    
    # Update step plan based on dynamic obstacles
    update_footstep_plan(feedback.current_pose)
```

---