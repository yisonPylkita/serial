project(wjwwood_serial_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(serial_example serial_example.cc stl_utils.hpp)
target_link_libraries(serial_example PRIVATE wjwwood::serial)

#add_executable(device_manager device_manager.cpp stl_utils.hpp DeviceImpl_CRTP.hpp)
#target_link_libraries(device_manager PRIVATE wjwwood::serial)


add_library(stdc++fs UNKNOWN IMPORTED)
set_property(TARGET stdc++fs PROPERTY IMPORTED_LOCATION "/usr/lib/gcc/x86_64-linux-gnu/7/libstdc++fs.a")

add_executable(new_way new_way.cpp)
target_link_libraries(new_way PRIVATE stdc++fs)
target_link_libraries(new_way PRIVATE wjwwood::serial)