# Chapter 2: ROS 2 Communication Primitives

## The Language of Robots: Nodes, Topics, Services, and Actions

At its heart, ROS 2 is about communication. Imagine a robot's "brain" as a collection of independent programs, each focused on a specific task: one node processes camera data, another controls motors, another plans movements. These are the building blocks of ROS 2 systems.

### Nodes and the Computation Graph

A **node** is an executable that performs a specific function in the ROS 2 system. Nodes are isolated processes that communicate with other nodes through ROS 2's middleware. This isolation provides robustness—if one node crashes, others continue functioning.

The **computation graph** is the network of all active nodes and the connections between them. Visualizing this graph using tools like `rqt_graph` helps you understand your robotic system's architecture.

```
Camera Node → Image Processing → Motion Planning → Motor Control Node
   ↓              ↓                    ↓
/camera/image  /processed/img    /trajectory
```

### Topics: Continuous Data Streams

**Topics** are named channels over which nodes exchange data asynchronously. A node **publishes** messages to a topic, while other nodes **subscribe** to that same topic to receive those messages.

Characteristics of topics:
- **One-to-Many**: One publisher, many subscribers
- **Asynchronous**: Publishers don't wait for subscribers to receive data
- **Continuous**: Suitable for sensor data, status updates, control commands
- **Loosely Coupled**: Publisher and subscriber don't need to know each other

Use case: A humanoid's camera node publishes images at 30 Hz. Multiple nodes (object detection, motion planning, visualization) can subscribe independently.

### Services: Request-Response Communication

**Services** implement synchronous, request-response communication. A client node sends a request to a server node and waits for a response. The client is blocked until the server responds.

Characteristics of services:
- **Two-way Communication**: Request and response
- **Synchronous**: Client waits for response
- **Short-lived**: Meant for quick operations
- **One-to-One**: Typically one server, multiple clients

Use case: A humanoid's arm controller provides a service `/move_arm_to_pose` that clients can call to request a specific arm position.

### Actions: Long-Running Tasks with Feedback

**Actions** are designed for long-running, goal-oriented tasks that provide continuous feedback. An action client sends a **goal** to an action server, which then works toward achieving that goal while providing periodic **feedback** and finally a **result**.

Characteristics of actions:
- **Goal-Oriented**: Client specifies desired outcome
- **Feedback**: Server provides progress updates
- **Cancellable**: Client can cancel an ongoing goal
- **Result-Driven**: Final outcome returned to client

Use case: A humanoid receives an action goal `/navigate_to_kitchen`. The action server provides feedback every 0.5 seconds about distance traveled and ultimately returns whether the navigation succeeded.

### Communication Comparison

```
Topic:   Sensor1 →(continuous stream)→ [Filter] → [Planner]
Service: UI →(request: "Get status")→ StatusServer →(response: "Ready") → UI
Action:  Client →(Goal: "Walk 5m") → ActionServer →(Feedback: "1m, 2m...") → Result
```

---