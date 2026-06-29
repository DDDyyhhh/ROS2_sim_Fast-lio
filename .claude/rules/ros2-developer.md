---
name: ros2-developer
description: ROS 2 Humble C++ development rules — C++17, rclcpp::Node, QoS, defensive programming, CMake patterns
type: always
---

# ROS 2 Humble C++ Developer Rules for RK3588 (Orange Pi 5 Plus)

These rules govern all C++ node development and modification within this workspace.
They apply to `FAST_LIO_ROS2`, `outdoor_sim`, and any future ROS 2 packages.

## 1. C++ Language & Build Standards

### Language Standard
- **Use C++17** (`-std=c++17`). Set in CMake:
  ```cmake
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  if(CMAKE_CXX_STANDARD_REQUIRED)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
  endif()
  ```

### Node Structure
- ALL ROS 2 nodes MUST inherit from `rclcpp::Node`:
  ```cpp
  class MyNode : public rclcpp::Node {
  public:
    MyNode() : Node("my_node_name") { ... }
  };
  ```
- Use in-class initialization for member variables.
- Prefer constructor injection for dependencies.

### Smart Pointers
- **ALWAYS** use `std::shared_ptr` / `std::unique_ptr` for ROS 2 entities:
  ```cpp
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::shared_ptr<MyCustomClass> helper_;
  ```
- Never use raw pointers for ROS 2 publisher/subscription/timer handles.
- Use `std::make_shared` / `std::make_unique` — never `new` / `delete`.

### CMake Dependencies
- **MUST** use `ament_target_dependencies` to bind package dependencies:
  ```cmake
  ament_target_dependencies(my_node
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2_ros
    PCL
    Eigen3
  )
  ```
- Never manually hard-code `-I/path/to/include` or `-lros2_lib` flags.

### NEON Optimization (RK3588)
- For heavy math code (FAST-LIO matrix ops), add:
  ```cmake
  target_compile_options(my_node PRIVATE -O3 -march=armv8-a+crypto -mcpu=cortex-a76)
  ```

## 2. Communication & QoS Rules

### Default QoS Policy (by topic type)

| Topic Type | Reliability | Durability | History | Depth |
|-----------|-------------|------------|---------|-------|
| **Sensor data** (LiDAR, IMU, PointCloud2, LaserScan, Image) | `BEST_EFFORT` | `VOLATILE` | `KEEP_LAST` | 10 |
| **Clock** (`/clock`) | `BEST_EFFORT` | `VOLATILE` | `KEEP_LAST` | 1 |
| **TF** (`/tf`, `/tf_static`) | `RELIABLE` | `TRANSIENT_LOCAL` | `KEEP_LAST` | 100 |
| **Control / Commands** (`/cmd_vel`, etc.) | `RELIABLE` | `VOLATILE` | `KEEP_LAST` | 10 |
| **Service** (request/response) | `RELIABLE` (implied) | `VOLATILE` | N/A | N/A |

### QoS Implementation Pattern
```cpp
// Sensor subscriber — BEST EFFORT (LiDAR / IMU)
auto sensor_qos = rclcpp::QoS(10)
  .best_effort()
  .keep_last(10)
  .durability_volatile();

sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  "/velodyne_points", sensor_qos, callback);

// Control subscriber — RELIABLE
auto control_qos = rclcpp::QoS(10)
  .reliable()
  .keep_last(10)
  .durability_volatile();

cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "/cmd_vel", control_qos, callback);
```

### Critical Rule
- **LiDAR and IMU data streams MUST use `BEST_EFFORT`**. Using `RELIABLE` on high-frequency sensor data causes:
  - DDS buffer overflow on RK3588's limited bandwidth.
  - Stale data delivery → SLAM divergence.
  - Blocking publisher loops in Ignition bridge.

## 3. Defensive Programming & Exception Safety

### Null Pointer Checks
Before dereferencing any pointer from external data (sensor callbacks, service requests, TF lookups), check for null:
```cpp
void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!msg) {
    RCLCPP_ERROR(this->get_logger(), "Received null PointCloud2 message");
    return;
  }
  if (msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud, skipping frame");
    return;
  }
  // ... process msg->data ...
}
```

### TF Lookup Safety
```cpp
geometry_msgs::msg::TransformStamped transform;
try {
  transform = tf_buffer_->lookupTransform("map", "lidar_link", tf2::TimePointZero);
} catch (const tf2::TransformException &ex) {
  RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
  return;  // Skip this frame gracefully
}
```

### Boundary & Overflow Checks
- Before accessing array/vector indices, validate bounds:
  ```cpp
  if (index >= points.size()) {
    RCLCPP_ERROR(this->get_logger(), "Point index %zu out of bounds (size=%zu)", index, points.size());
    return;
  }
  ```
- When converting between integer types (e.g., `size_t` → `int`), check overflow:
  ```cpp
  if (size > static_cast<size_t>(std::numeric_limits<int>::max())) {
    RCLCPP_ERROR(this->get_logger(), "Size overflow: %zu exceeds int max", size);
    return;
  }
  ```

### Sensor Data Parser Safety
- PointCloud2 field offsets — always validate before use:
  ```cpp
  // Validate that required fields exist
  auto x_field = msg->fields.at(0);  // may throw std::out_of_range
  if (x_field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
    RCLCPP_ERROR(this->get_logger(), "Unexpected point field type");
    return;
  }
  ```

### RK3588-Specific: SIMD Alignment
- When using ARM NEON intrinsics, ensure data is 16-byte aligned:
  ```cpp
  // Prefer Eigen::aligned_allocator for std::vector of Eigen types
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> matrices;
  ```

## 4. Code Style
- **Naming**: `snake_case` for functions/variables/files; `CamelCase` for classes.
- **Logging**: Use `RCLCPP_*` macros (not raw `std::cout` or `printf`).
- **Comments**: Document WHY, not WHAT. Chinese comments allowed in this codebase.
- **Error recovery**: Nodes should degrade gracefully — a single bad sensor frame must NOT crash the process (no uncaught exceptions, no `std::abort`).

## 5. Testing Before Launch
Before running full SLAM, always verify:
1. `ros2 topic list` — confirm `/velodyne_points`, `/imu/data`, `/clock` exist.
2. `ros2 topic echo /clock` — confirm simulation time advances (not stuck at 0.00).
3. `ros2 run tf2_tools view_frames` — confirm TF tree integrity.
4. `ros2 topic hz /velodyne_points` — confirm LiDAR at ~10 Hz.
5. `ros2 topic hz /imu/data` — confirm IMU at ~200 Hz.
