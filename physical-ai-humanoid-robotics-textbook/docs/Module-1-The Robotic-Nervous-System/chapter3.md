# Chapter 3: Implementing Your First ROS 2 System

## Hands-On: Building a Humanoid Control Architecture

Let's transition from theory to practice. We'll build a complete ROS 2 system where an AI agent (simulated LLM) communicates with motor controllers through ROS 2 primitives.

### Setting Up Your ROS 2 Workspace

```bash
# Create workspace directory structure
mkdir -p ~/ros2_humanoid_ws/src
cd ~/ros2_humanoid_ws/src

# Create a new ROS 2 Python package
ros2 pkg create --build-type ament_python humanoid_controller

# Navigate into the package
cd humanoid_controller
mkdir -p humanoid_controller launch urdf rviz
```

### Chapter 3.1: Building Your First Publisher Node

Create `humanoid_controller/arm_command_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class ArmCommandNode(Node):
    def __init__(self):
        super().__init__('arm_command_node')
        self.publisher_ = self.create_publisher(
            String, 
            'arm_commands', 
            10
        )
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.publish_command)
        self.get_logger().info('Arm Command Node initialized')
        
    def publish_command(self):
        commands = ['wave', 'reach_forward', 'grab', 'rest', 'point']
        selected = random.choice(commands)
        msg = String()
        msg.data = f'COMMAND:{selected}:TIMESTAMP:{time.time()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Chapter 3.2: Creating a Subscriber + Publisher Bridge

Create `humanoid_controller/motor_controller_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Subscribe to arm commands
        self.subscription = self.create_subscription(
            String,
            'arm_commands',
            self.command_callback,
            10
        )
        
        # Publish motor signals
        self.motor_publisher = self.create_publisher(
            Float32MultiArray,
            'motor_signals',
            10
        )
        
        self.get_logger().info('Motor Controller Node ready')
    
    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        
        # Parse command and translate to motor signals
        if 'wave' in msg.data:
            self.execute_wave()
        elif 'reach_forward' in msg.data:
            self.execute_reach_forward()
        elif 'grab' in msg.data:
            self.execute_grab()
    
    def execute_wave(self):
        motor_msg = Float32MultiArray()
        motor_msg.data = [45.0, 30.0, 20.0]  # Example: shoulder, elbow, wrist angles
        self.motor_publisher.publish(motor_msg)
        self.get_logger().info('Executing: Wave motion')
    
    def execute_reach_forward(self):
        motor_msg = Float32MultiArray()
        motor_msg.data = [0.0, 90.0, 0.0]
        self.motor_publisher.publish(motor_msg)
        self.get_logger().info('Executing: Reach forward')
    
    def execute_grab(self):
        motor_msg = Float32MultiArray()
        motor_msg.data = [30.0, 60.0, -45.0]
        self.motor_publisher.publish(motor_msg)
        self.get_logger().info('Executing: Grab object')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Chapter 3.3: Service Implementation - Status Requests

Create `humanoid_controller/status_service_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class StatusServiceNode(Node):
    def __init__(self):
        super().__init__('status_service_node')
        self.service = self.create_service(
            Trigger,
            'get_robot_status',
            self.status_callback
        )
        self.robot_ready = True
        self.get_logger().info('Status Service Node ready')
    
    def status_callback(self, request, response):
        response.success = self.robot_ready
        response.message = f'Robot Status: {"READY" if self.robot_ready else "ERROR"}'
        self.get_logger().info(f'Status requested: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StatusServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Chapter 3.4: Building and Running

Update `setup.py`:

```python
from setuptools import setup, find_packages

setup(
    name='humanoid_controller',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/humanoid_controller', ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'arm_command = humanoid_controller.arm_command_node:main',
            'motor_controller = humanoid_controller.motor_controller_node:main',
            'status_service = humanoid_controller.status_service_node:main',
        ],
    },
)
```

Build and test:

```bash
cd ~/ros2_humanoid_ws
colcon build
source install/setup.bash

# Terminal 1: Run arm command publisher
ros2 run humanoid_controller arm_command

# Terminal 2: Run motor controller subscriber
ros2 run humanoid_controller motor_controller

# Terminal 3: Monitor communication
ros2 topic echo arm_commands
```

---