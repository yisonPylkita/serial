project(wjwwood_serial_examples)

set(CMAKE_CXX_STANDARD 17)

add_executable(serial_example serial_example.cc stl_utils.hpp)
target_link_libraries(serial_example PRIVATE wjwwood::serial)

add_executable(device_manager device_manager.cpp stl_utils.hpp)
target_link_libraries(device_manager PRIVATE wjwwood::serial)