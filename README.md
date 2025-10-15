# Ryder Robot Buffer Services

Various buffer nodes.

## Development Notes

To test code without deploying

```bash
mkdir -p ~/rr_quadx_ws
cd ~/rr_quadx_ws
mkdir -p src
cd src
git clone https://github.com/Ryder-Robots/rr_buffer_services.git
colcon build --packages-select rr_buffer_services
source install/local_setup.bash

# To show added messages and actions
ros2 interface list | grep rr_buffer_services
ros2 run rr_buffer_services rrbuffer --ros-args --log-level DEBUG

```

## Running Tests

```bash
colcon build --packages-up-to rr_buffer_services --cmake-args -DBUILD_TESTING=On --allow-overriding rtest
colcon test-result --verbose
```

## References

* [Writing an action server and client](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-client)
* [Writing Basic Tests with C++ with GTest](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Cpp.html)
