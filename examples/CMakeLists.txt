project(wjwwood_serial_examples)

add_executable(${PROJECT_NAME} serial_example.cc)
target_link_libraries(${PROJECT_NAME} PRIVATE wjwwood::serial)