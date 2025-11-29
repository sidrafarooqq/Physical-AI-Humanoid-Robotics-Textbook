# Chapter 3: From Planning to Execution - ROS 2 Action Integration

### 3.1 ROS 2 Actions Architecture

#### 3.1.1 Action-Based Task Execution

```python
# ROS 2 Action Server for Robot Tasks
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Define custom action messages
class ExecuteRobotTask(Action):
    """Custom action for robot task execution"""
    
    class Goal:
        action_name: str
        parameters: Dict[str, str]
    
    class Result:
        success: bool
        message: str
    
    class Feedback:
        progress: float  # 0.0 to 1.0
        status: str

class RobotTaskExecutor(Node):
    """Execute planned actions via ROS 2 actions"""
    
    def __init__(self):
        super().__init__('robot_task_executor')
        
        # Create action server
        self._action_server = ActionServer(
            self,
            ExecuteRobotTask,
            'execute_task',
            self.execute_callback
        )
        
        # Action clients for robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.manipulation_client = ActionClient(
            self, 
            PickPlace, 
            '/pick_place'
        )
        
        self.get_logger().info("🤖 Robot Task Executor initialized")
    
    async def execute_callback(self, goal_handle):
        """Execute a single planned action"""
        
        goal = goal_handle.request
        action_name = goal.action_name
        parameters = goal.parameters
        
        self.get_logger().info(f"Executing: {action_name}")
        
        try:
            # Execute based on action type
            if action_name == 'navigate':
                success = await self._execute_navigate(
                    parameters['destination'],
                    goal_handle
                )
            
            elif action_name == 'pick_up':
                success = await self._execute_pick_up(
                    parameters['object'],
                    goal_handle
                )
            
            elif action_name == 'place':
                success = await self._execute_place(
                    parameters['object'],
                    parameters['location'],
                    goal_handle
                )
            
            elif action_name == 'look_for':
                success = await self._execute_look_for(
                    parameters['target'],
                    goal_handle
                )
            
            else:
                success = False
                self.get_logger().warn(f"Unknown action: {action_name}")
            
            # Set result
            if success:
                goal_handle.succeed()
                result = ExecuteRobotTask.Result()
                result.success = True
                result.message = f"Successfully executed {action_name}"
            else:
                goal_handle.abort()
                result = ExecuteRobotTask.Result()
                result.success = False
                result.message = f"Failed to execute {action_name}"
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Action execution error: {e}")
            goal_handle.abort()
            result = ExecuteRobotTask.Result()
            result.success = False
            result.message = str(e)
            return result
    
    async def _execute_navigate(self, destination: str, 
                               goal_handle) -> bool:
        """Navigate to destination"""
        
        # Convert destination name to coordinates
        coords = self._location_to_coordinates(destination)
        
        if coords is None:
            return False
        
        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.pose.position.x = coords[0]
        nav_goal.pose.pose.position.y = coords[1]
        
        # Wait for nav2 server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return False
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        
        # Wait for result
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_result = send_goal_future.result()
        
        if goal_result is None:
            return False
        
        # Get result
        get_result_future = goal_result.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        # Update feedback
        feedback = ExecuteRobotTask.Feedback()
        feedback.progress = 1.0
        feedback.status = f"Navigated to {destination}"
        goal_handle.publish_feedback(feedback)
        
        return get_result_future.result().success
    
    async def _execute_pick_up(self, obj: str, goal_handle) -> bool:
        """Pick up object"""
        
        # Use vision to locate object
        object_pose = await self._locate_object(obj)
        
        if object_pose is None:
            self.get_logger().warn(f"Object '{obj}' not found")
            return False
        
        # Create manipulation goal
        manip_goal = PickPlace.Goal()
        manip_goal.object_name = obj
        manip_goal.target_pose = object_pose
        manip_goal.action = "pick_up"
        
        # Send goal
        send_goal_future = self.manipulation_client.send_goal_async(manip_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_result = send_goal_future.result()
        if goal_result is None:
            return False
        
        # Update feedback
        feedback = ExecuteRobotTask.Feedback()
        feedback.progress = 1.0
        feedback.status = f"Picked up {obj}"
        goal_handle.publish_feedback(feedback)
        
        return True
    
    async def _execute_place(self, obj: str, location: str, 
                            goal_handle) -> bool:
        """Place object at location"""
        
        # Get placement coordinates
        placement_coords = self._location_to_coordinates(location)
        
        if placement_coords is None:
            return False
        
        # Create manipulation goal
        manip_goal = PickPlace.Goal()
        manip_goal.object_name = obj
        manip_goal.target_pose = placement_coords
        manip_goal.action = "place"
        
        # Send goal
        send_goal_future = self.manipulation_client.send_goal_async(manip_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        feedback = ExecuteRobotTask.Feedback()
        feedback.progress = 1.0
        feedback.status = f"Placed {obj} at {location}"
        goal_handle.publish_feedback(feedback)
        
        return True
    
    async def _execute_look_for(self, target: str, goal_handle) -> bool:
        """Search for object"""
        
        target_pose = await self._locate_object(target)
        
        feedback = ExecuteRobotTask.Feedback()
        feedback.progress = 1.0
        feedback.status = f"Found {target}" if target_pose else f"Could not find {target}"
        goal_handle.publish_feedback(feedback)
        
        return target_pose is not None
    
    def _location_to_coordinates(self, location: str):
        """Convert location name to coordinates"""
        location_map = {
            'living_room': (0.0, 0.0),
            'kitchen': (5.0, 0.0),
            'bedroom': (0.0, 5.0),
            'bathroom': (5.0, 5.0),
            'hallway': (2.5, 2.5)
        }
        return location_map.get(location.lower())
    
    async def _locate_object(self, obj: str):
        """Use vision to locate object"""
        # This would integrate with vision/detection modules
        pass
```

### 3.2 Plan Execution Pipeline

#### 3.2.1 Complete Execution Loop

```python
class PlanExecutor(Node):
    """Execute complete plans from LLM"""
    
    def __init__(self):
        super().__init__('plan_executor')
        
        self.planner = CognitivePlanner()
        self.task_executor = RobotTaskExecutor()
        self.executor = MultiThreadedExecutor()
        
        # Subscribe to voice commands
        self.command_subscription = self.create_subscription(
            String,
            '/robot/voice_command',
            self.voice_command_callback,
            10
        )
        
        # Publish execution status
        self.status_publisher = self.create_publisher(
            String,
            '/robot/execution_status',
            10
        )
    
    def voice_command_callback(self, msg: String):
        """Handle incoming voice command"""
        
        command = msg.data
        self.get_logger().info(f"🗣️ Voice command: {command}")
        
        # Generate plan
        self.get_logger().info("🧠 Generating plan...")
        plan = self.planner.plan_task(command)
        
        # Validate plan
        is_valid, reason = self.planner.validate_plan(plan)
        
        if not is_valid:
            self.get_logger().error(f"Invalid plan: {reason}")
            status_msg = String()
            status_msg.data = f"Error: Invalid plan - {reason}"
            self.status_publisher.publish(status_msg)
            return
        
        self.get_logger().info(f"Plan validated with {len(plan)} steps")
        
        # Execute plan
        self.get_logger().info("⚡ Executing plan...")
        success, message = self.execute_plan(plan)
        
        # Publish result
        status_msg = String()
        status_msg.data = f"Execution {'successful' if success else 'failed'}: {message}"
        self.status_publisher.publish(status_msg)
        
        if success:
            self.get_logger().info("✅ Task completed successfully!")
        else:
            self.get_logger().error(f"❌ Task failed: {message}")
    
    def execute_plan(self, plan: List[Dict]) -> Tuple[bool, str]:
        """Execute a complete plan"""
        
        for step_idx, step in enumerate(plan):
            self.get_logger().info(
                f"Step {step_idx + 1}/{len(plan)}: {step['action']}"
            )
            
            try:
                # Create and execute action
                success = self._execute_single_action(step)
                
                if not success:
                    return False, f"Failed at step {step_idx + 1}"
                
            except Exception as e:
                self.get_logger().error(f"Exception in step {step_idx}: {e}")
                return False, str(e)
        
        return True, "All steps completed"
    
    def _execute_single_action(self, action: Dict) -> bool:
        """Execute single action from plan"""
        # Implementation delegates to action servers
        pass
```

---