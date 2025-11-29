# Chapter 3: Sensor Simulation and Data Acquisition

### 3.1 LiDAR (Light Detection and Ranging)

LiDAR sensors measure distance by emitting laser beams and analyzing reflections. Gazebo simulates LiDAR with the following capabilities:

**Key Parameters:**
- **Horizontal Resolution**: Number of beams in horizontal plane (typically 720-1440)
- **Vertical Resolution**: Number of vertical scans (typically 16-64)
- **Range**: Minimum and maximum detection distance (5-200m)
- **Update Rate**: Frequency of scan generation (5-20 Hz)

**Simulated Output:**
```
Point Cloud Data:
(x, y, z, intensity)
Range from [0.0m, 30.0m]
```

### 3.2 Depth Cameras

Depth cameras provide 3D spatial information using structured light or time-of-flight technology. In Gazebo simulation:

**Parameters:**
- **Resolution**: Typical 640×480 or 1280×720 pixels
- **Field of View**: Usually 60-90 degrees
- **Depth Range**: 0.3-10 meters
- **Frame Rate**: 30-60 fps

**Data Acquisition:**
Depth cameras output a depth image where each pixel represents distance from the camera, allowing for obstacle detection and environment mapping.

### 3.3 Inertial Measurement Units (IMUs)

IMUs measure acceleration and angular velocity, providing crucial feedback for robot balance and orientation.

**Simulated Measurements:**
- **Linear Acceleration**: x, y, z components (m/s²)
- **Angular Velocity**: Roll, pitch, yaw rates (rad/s)
- **Orientation**: Quaternion or Euler angles
- **Noise Models**: Gaussian noise added to simulate real sensors

```python
# IMU data structure in simulation
imu_data = {
    'linear_acceleration': [0.0, 0.0, 9.81],  # m/s²
    'angular_velocity': [0.0, 0.0, 0.0],      # rad/s
    'orientation': [0.0, 0.0, 0.0, 1.0],      # quaternion
    'timestamp': 1234567890.123               # seconds
}
```

### 3.4 Sensor Fusion and Data Processing

Multiple sensors provide complementary information. Sensor fusion techniques combine data from LiDAR, depth cameras, and IMUs for robust environment perception:

1. **Data Synchronization**: Aligning sensor data timestamps
2. **Filtering**: Removing noise and outliers
3. **Registration**: Aligning point clouds from multiple sensors
4. **Feature Extraction**: Identifying objects and surfaces in the environment

---
