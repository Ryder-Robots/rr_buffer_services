# Ryder Robot Buffer Services

## Development Notes

To test code without deploying

```bash
colcon build --packages-select rr_buffer_services
source install/local_setup.bash

# To show added messages and actions
ros2 interface list | grep rr_buffer_services
```

## Running Tests

```bash
colcon test --ctest-args tests --packages-up-to  rr_buffer_services
```

## References

* [Creating an action](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Creating-an-Action.html)
* [Writing an action server and client](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-client)
* [Writing Basic Tests with C++ with GTest](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Cpp.html)
