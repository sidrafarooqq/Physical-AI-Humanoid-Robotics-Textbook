# Chapter 2: Cognitive Planning - LLMs Translating Language to Action

### 2.1 Large Language Models in Robotics

Large Language Models (LLMs) like GPT-4, Claude, and Llama excel at understanding context, reasoning about tasks, and generating structured outputs. This capability transforms how robots plan complex tasks.

**LLM Advantages for Robotics:**
- **Common Sense Reasoning**: Understands real-world constraints
- **Task Decomposition**: Breaks complex goals into steps
- **Error Recovery**: Generates backup plans
- **Multi-step Planning**: Coordinates multiple robot actions
- **Natural Language Interface**: Accepts human descriptions

### 2.2 Cognitive Planning Architecture

#### 2.2.1 System Overview

```
User: "Clean the room"
     ↓
┌─────────────────────────┐
│ Voice Recognition       │ → "clean the room"
│ (Whisper)               │
└─────────────────────────┘
     ↓
┌─────────────────────────┐
│ Intent Recognition      │ → Intent: "clean"
│ (Command Parser)        │    Objects: "room"
└─────────────────────────┘
     ↓
┌─────────────────────────┐
│ Cognitive Planning      │ → Plan: [
│ (LLM)                   │    1. Map room
│                         │    2. Identify dirt
│                         │    3. Pick up trash
│                         │    4. Return to start
└─────────────────────────┘
     ↓
┌─────────────────────────┐
│ Task Execution          │ → Execute ROS 2
│ (ROS 2 Action Servers)  │    actions
└─────────────────────────┘
```

#### 2.2.2 Prompt Engineering for Task Planning

```python
# LLM-based Task Planner
import openai
from typing import List, Dict

class CognitivePlanner:
    """Use LLM for task planning"""
    
    def __init__(self, model="gpt-4"):
        self.model = model
        self.api_key = os.getenv('OPENAI_API_KEY')
        openai.api_key = self.api_key
        
        # System prompt defining robot capabilities
        self.system_prompt = """
You are an AI planning assistant for a humanoid robot. 
The robot has the following capabilities:
- navigate(destination: str) - Move to a named location
- pick_up(object: str) - Grab an object
- place(object: str, location: str) - Place object at location
- look_for(object: str) - Search for an object
- open(target: str) - Open door/drawer
- close(target: str) - Close door/drawer
- move_arm(target_pose: str) - Move arm to target
- gripper_open() / gripper_close() - Control gripper
- wait(seconds: float) - Wait/pause

Current environment:
- Locations: living_room, kitchen, bedroom, bathroom, hallway
- Objects: trash, dishes, books, clothes, dust
- Sensors: RGB camera, depth camera, LIDAR, IMU

When given a task:
1. Break it down into atomic actions
2. Use only available capabilities
3. Consider dependencies between actions
4. Include safety checks
5. Return plan as JSON array

Always respond with valid JSON only, no additional text.
"""
    
    def plan_task(self, task_description: str) -> List[Dict]:
        """
        Generate task plan from natural language description
        
        Args:
            task_description: What user wants robot to do
            
        Returns:
            List of atomic actions with parameters
        """
        user_prompt = f"""
Plan the following task for the humanoid robot:
"{task_description}"

Return the plan as a JSON array where each element is:
{{
    "action": "action_name",
    "parameters": {{"param1": "value1", "param2": "value2"}},
    "description": "What this step does"
}}

Example for "Pick up the book":
[
    {{"action": "look_for", "parameters": {{"object": "book"}}, "description": "Find the book"}},
    {{"action": "navigate", "parameters": {{"destination": "book location"}}, "description": "Move to book"}},
    {{"action": "pick_up", "parameters": {{"object": "book"}}, "description": "Grasp the book"}}
]
"""
        
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,  # Lower for more deterministic planning
                max_tokens=2000
            )
            
            # Extract plan from response
            plan_text = response['choices'][0]['message']['content']
            
            # Parse JSON
            import json
            plan = json.loads(plan_text)
            
            return plan
            
        except json.JSONDecodeError as e:
            print(f"Failed to parse LLM response as JSON: {e}")
            return []
        except openai.error.OpenAIError as e:
            print(f"OpenAI API error: {e}")
            return []
    
    def validate_plan(self, plan: List[Dict]) -> Tuple[bool, str]:
        """
        Validate that plan is executable
        
        Args:
            plan: Generated task plan
            
        Returns:
            (is_valid, reason)
        """
        valid_actions = {
            'navigate', 'pick_up', 'place', 'look_for', 
            'open', 'close', 'move_arm', 'gripper_open', 
            'gripper_close', 'wait'
        }
        
        if not isinstance(plan, list):
            return False, "Plan must be a list of actions"
        
        if len(plan) == 0:
            return False, "Plan is empty"
        
        if len(plan) > 50:
            return False, "Plan too long (max 50 steps)"
        
        for i, step in enumerate(plan):
            # Check action exists
            if 'action' not in step:
                return False, f"Step {i}: Missing 'action' field"
            
            action = step['action']
            if action not in valid_actions:
                return False, f"Step {i}: Unknown action '{action}'"
            
            # Check parameters exist
            if 'parameters' not in step:
                return False, f"Step {i}: Missing 'parameters' field"
        
        return True, "Plan is valid"
```

#### 2.2.3 Hierarchical Planning with Subtasks

```python
class HierarchicalPlanner:
    """Support multi-level task planning"""
    
    def __init__(self):
        self.base_planner = CognitivePlanner()
        self.subtask_cache = {}
    
    def decompose_complex_task(self, task: str, depth: int = 0) -> List[Dict]:
        """
        Recursively decompose complex tasks
        
        Args:
            task: High-level task description
            depth: Current recursion depth
            
        Returns:
            Fully decomposed action plan
        """
        if depth > 3:  # Prevent infinite recursion
            return []
        
        # Get initial plan
        plan = self.base_planner.plan_task(task)
        
        # Check if plan contains abstract actions
        abstract_actions = []
        for i, step in enumerate(plan):
            if self._is_abstract(step['action']):
                abstract_actions.append((i, step))
        
        # If no abstract actions, return plan
        if not abstract_actions:
            return plan
        
        # Decompose abstract actions
        expanded_plan = []
        for i, (index, step) in enumerate(abstract_actions):
            # Add all steps before this abstract action
            if i == 0:
                expanded_plan.extend(plan[:index])
            else:
                prev_index = abstract_actions[i-1][0]
                expanded_plan.extend(plan[prev_index+1:index])
            
            # Recursively decompose abstract action
            subtask = f"{step['action']}({', '.join(step['parameters'].values())})"
            subtask_plan = self.decompose_complex_task(subtask, depth + 1)
            expanded_plan.extend(subtask_plan)
        
        # Add remaining steps
        if abstract_actions:
            last_index = abstract_actions[-1][0]
            expanded_plan.extend(plan[last_index+1:])
        
        return expanded_plan
    
    def _is_abstract(self, action: str) -> bool:
        """Check if action is abstract (needs decomposition)"""
        abstract_actions = {'clean', 'tidy', 'organize', 'prepare'}
        return action in abstract_actions
```

### 2.3 Context-Aware Planning

#### 2.3.1 Incorporating Environmental Context

```python
class ContextualPlanner:
    """Plan with awareness of environment state"""
    
    def __init__(self):
        self.planner = CognitivePlanner()
        self.world_state = {}
        self.robot_state = {}
    
    def update_world_state(self, perception_data: Dict):
        """Update understanding of environment"""
        self.world_state.update(perception_data)
    
    def update_robot_state(self, state_data: Dict):
        """Update robot status (battery, position, etc)"""
        self.robot_state.update(state_data)
    
    def plan_with_context(self, task: str) -> List[Dict]:
        """
        Generate plan considering current state
        
        Args:
            task: Task description
            
        Returns:
            Context-aware plan
        """
        # Build context prompt
        context = f"""
Current Robot State:
- Position: {self.robot_state.get('position', 'unknown')}
- Battery: {self.robot_state.get('battery', 'unknown')}%
- Gripper: {'open' if self.robot_state.get('gripper_open') else 'closed'}
- Current task: {self.robot_state.get('current_task', 'idle')}

World State:
- Detected objects: {self.world_state.get('objects', [])}
- Obstacles: {self.world_state.get('obstacles', [])}
- Door status: {self.world_state.get('doors', {})}
"""
        
        full_prompt = f"{context}\n\nTask: {task}"
        
        return self.planner.plan_task(full_prompt)
```

### 2.4 Error Handling and Replanning

#### 2.4.1 Failure Detection and Recovery

```python
class AdaptivePlanner:
    """Handle plan failures and adapt"""
    
    def __init__(self):
        self.planner = CognitivePlanner()
        self.current_plan = []
        self.execution_history = []
    
    def execute_with_recovery(self, plan: List[Dict], 
                             max_retries: int = 3) -> Tuple[bool, str]:
        """
        Execute plan with automatic recovery on failure
        
        Args:
            plan: Action plan to execute
            max_retries: Times to retry on failure
            
        Returns:
            (success, final_status)
        """
        self.current_plan = plan
        
        for step_idx, step in enumerate(plan):
            success = False
            
            for retry in range(max_retries):
                try:
                    # Execute step
                    result = self._execute_action(step)
                    
                    # Log execution
                    self.execution_history.append({
                        'step': step_idx,
                        'action': step['action'],
                        'status': 'success',
                        'timestamp': time.time()
                    })
                    
                    success = True
                    break
                    
                except ActionFailureException as e:
                    self.execution_history.append({
                        'step': step_idx,
                        'action': step['action'],
                        'status': 'failed',
                        'error': str(e),
                        'retry': retry,
                        'timestamp': time.time()
                    })
                    
                    if retry < max_retries - 1:
                        # Wait before retry
                        time.sleep(2 ** retry)  # Exponential backoff
            
            if not success:
                # Generate recovery plan
                recovery_plan = self._generate_recovery_plan(
                    step, 
                    self.execution_history
                )
                
                if recovery_plan:
                    # Execute recovery plan
                    _, recovery_status = self.execute_with_recovery(
                        recovery_plan, 
                        max_retries=1
                    )
                else:
                    return False, f"Failed at step {step_idx}: {step['action']}"
        
        return True, "Plan executed successfully"
    
    def _generate_recovery_plan(self, failed_step: Dict, 
                               history: List[Dict]) -> List[Dict]:
        """Generate recovery plan for failed action"""
        
        recovery_prompt = f"""
The robot failed to execute the following action:
{failed_step['action']}({', '.join(failed_step['parameters'].values())})

Recent execution history:
{history[-5:]}

Generate an alternative plan to recover from this failure.
Consider different approaches or intermediate steps needed.
"""
        
        recovery_plan = self.planner.plan_task(recovery_prompt)
        return recovery_plan
```

---