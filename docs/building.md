# Building — Python Bindings & ROS 2 Bridge

## Build with Python bindings

```bash
pip install pybind11
cmake -B build -DTHUNDERBIRD_BUILD_PYTHON=ON
cmake --build build --parallel

# Run
cd python
PYTHONPATH=../build/python python example.py
```

## Build the ROS 2 bridge

```bash
# Inside a ROS 2 workspace
ln -s /path/to/thunderbird/ros2_bridge src/thunderbird_ros2_bridge
colcon build --packages-select thunderbird_ros2_bridge
source install/setup.bash
ros2 run thunderbird_ros2_bridge thunderbird_ros2_node
```
