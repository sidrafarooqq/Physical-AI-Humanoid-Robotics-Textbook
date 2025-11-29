# Chapter 4: Humanoid Modeling with URDF and Xacro

## From Concepts to Robot Models

A humanoid robot is defined by its kinematic structure—where joints are, what they can do, and how links (rigid bodies) connect. URDF (Unified Robot Description Format) is the XML-based standard for defining this structure in ROS 2.

### Understanding URDF Fundamentals

URDF documents contain:
- **Links**: Rigid bodies with visual geometry and collision properties
- **Joints**: Connections between links with motion constraints
- **Transmissions**: Mappings between joints and actuators
- **Materials**: Visual appearance properties

### Xacro: Making URDF Modular

**Xacro** (XML Macros) extends URDF with programming constructs: variables, macros, conditional logic. This dramatically reduces redundancy in complex models.

```xml
<xacro:property name="M_PI" value="3.14159265" />
<xacro:macro name="arm_segment" params="name length mass">
  <!-- Reusable arm segment definition -->
</xacro:macro>
```

### A 12-DoF Humanoid Model

Create `urdf/humanoid_12dof.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_12dof">

  <xacro:property name="M_PI" value="3.14159265" />
  <xacro:property name="BASE_MASS" value="5.0" />
  <xacro:property name="ARM_MASS" value="1.0" />
  <xacro:property name="LEG_MASS" value="2.5" />

  <xacro:macro name="default_inertial" params="mass">
    <inertial>
      <mass value="${mass}" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" 
               iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
  </xacro:macro>

  <!-- Base/Pelvis -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.2 0.2 0.3"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.2 0.2 0.3"/></geometry>
    </collision>
    <xacro:default_inertial mass="${BASE_MASS}"/>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
  <link name="torso_link">
    <visual>
      <geometry><box size="0.25 0.18 0.35"/></geometry>
      <material name="red"><color rgba="1 0 0 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.25 0.18 0.35"/></geometry>
    </collision>
    <xacro:default_inertial mass="10.0"/>
  </link>

  <!-- ARM MACRO -->
  <xacro:macro name="arm" params="side pos_y">
    <!-- Shoulder Joint -->
    <joint name="${side}_shoulder_pitch" type="revolute">
      <parent link="torso_link"/>
      <child link="${side}_shoulder_link"/>
      <origin xyz="0 ${pos_y} 0.1" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1.57"/>
    </joint>
    <link name="${side}_shoulder_link">
      <visual>
        <geometry><cylinder radius="0.05" length="0.1"/></geometry>
        <material name="green"><color rgba="0 1 0 1"/></material>
      </visual>
      <collision>
        <geometry><cylinder radius="0.05" length="0.1"/></geometry>
      </collision>
      <xacro:default_inertial mass="${ARM_MASS}"/>
    </link>

    <!-- Elbow Joint -->
    <joint name="${side}_elbow_pitch" type="revolute">
      <parent link="${side}_shoulder_link"/>
      <child link="${side}_forearm_link"/>
      <origin xyz="0.15 0 0" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="0" upper="${M_PI}" effort="100" velocity="1.57"/>
    </joint>
    <link name="${side}_forearm_link">
      <visual>
        <geometry><cylinder radius="0.04" length="0.15"/></geometry>
        <material name="yellow"><color rgba="1 1 0 1"/></material>
      </visual>
      <collision>
        <geometry><cylinder radius="0.04" length="0.15"/></geometry>
      </collision>
      <xacro:default_inertial mass="${ARM_MASS * 0.8}"/>
    </link>

    <!-- Wrist Joint -->
    <joint name="${side}_wrist_pitch" type="revolute">
      <parent link="${side}_forearm_link"/>
      <child link="${side}_hand_link"/>
      <origin xyz="0.15 0 0" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="50" velocity="2.0"/>
    </joint>
    <link name="${side}_hand_link">
      <visual>
        <geometry><box size="0.08 0.1 0.08"/></geometry>
        <material name="white"><color rgba="1 1 1 1"/></material>
      </visual>
      <collision>
        <geometry><box size="0.08 0.1 0.08"/></geometry>
      </collision>
      <xacro:default_inertial mass="0.3"/>
    </link>
  </xacro:macro>

  <!-- Instantiate both arms -->
  <xacro:arm side="right" pos_y="-0.12"/>
  <xacro:arm side="left" pos_y="0.12"/>

  <!-- LEG MACRO -->
  <xacro:macro name="leg" params="side pos_y">
    <!-- Hip Joint -->
    <joint name="${side}_hip_pitch" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_hip_link"/>
      <origin xyz="0 ${pos_y} -0.15" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="150" velocity="1.57"/>
    </joint>
    <link name="${side}_hip_link">
      <visual>
        <geometry><box size="0.1 0.08 0.12"/></geometry>
        <material name="orange"><color rgba="1 0.5 0 1"/></material>
      </visual>
      <collision>
        <geometry><box size="0.1 0.08 0.12"/></geometry>
      </collision>
      <xacro:default_inertial mass="${LEG_MASS}"/>
    </link>

    <!-- Knee Joint -->
    <joint name="${side}_knee_pitch" type="revolute">
      <parent link="${side}_hip_link"/>
      <child link="${side}_knee_link"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="0" upper="${M_PI/2}" effort="150" velocity="1.57"/>
    </joint>
    <link name="${side}_knee_link">
      <visual>
        <geometry><box size="0.1 0.08 0.2"/></geometry>
        <material name="purple"><color rgba="0.5 0 0.5 1"/></material>
      </visual>
      <collision>
        <geometry><box size="0.1 0.08 0.2"/></geometry>
      </collision>
      <xacro:default_inertial mass="${LEG_MASS}"/>
    </link>
  </xacro:macro>

  <!-- Instantiate both legs -->
  <xacro:leg side="right_leg" pos_y="-0.08"/>
  <xacro:leg side="left_leg" pos_y="0.08"/>

</robot>
```

### Visualization in RViz2

Create `launch/display.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('humanoid_controller')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'humanoid_12dof.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'default.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        ),
    ])
```

Launch and visualize:

```bash
cd ~/ros2_humanoid_ws
colcon build
source install/setup.bash
ros2 launch humanoid_controller display.launch.py
```

You should now see your 12-DoF humanoid in RViz2, with interactive sliders to move each joint!

---

## Summary: Your Robotic Nervous System is Ready

You've now mastered the foundational components of ROS 2:

- **Architecture**: Peer-to-peer DDS-based communication for resilience and scalability
- **Communication Primitives**: Topics for streams, Services for synchronous calls, Actions for long-running goals
- **Practical Implementation**: Building publisher/subscriber nodes, services, and bridges
- **Robot Modeling**: Defining complex humanoid structures with URDF and Xacro

Your humanoid robot now has a nervous system. What's next? In Module 2, we'll give it a **digital playground**—Gazebo and simulation environments where your robot can perceive, learn, and act in realistic physics-based worlds.

---

## Quick Reference: Common ROS 2 Commands

```bash
# List nodes, topics, services
ros2 node list
ros2 topic list
ros2 service list
ros2 action list

# Monitor data
ros2 topic echo /topic_name
ros2 topic pub /topic_name std_msgs/String "data: 'hello'"

# Inspect types
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show std_srvs/srv/Trigger

# Debugging
ros2 run rqt_graph rqt_graph
ros2 run rqt_console rqt_console

# Validate URDF
check_urdf humanoid_12dof.urdf.xacro
xacro humanoid_12dof.urdf.xacro > humanoid_12dof.urdf
```

---

## Further Reading & Resources

- **ROS 2 Official Docs**: https://docs.ros.org/en/humble/
- **ROS 2 Design**: https://design.ros2.org/
- **URDF/Xacro Tutorials**: https://wiki.ros.org/urdf/Tutorials
- **rclpy Documentation**: https://docs.ros.org/en/humble/p/rclpy/
- **DDS Specification**: https://www.omg.org/dds/