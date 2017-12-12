project(wjwwood_serial_tests)

set(WJWWOOD_SERIAL_UTESTS_SOURCES
    catch.hpp
    main.cpp
    unix_serial_tests.cc
    unix_timer_tests.cc)

add_executable(${PROJECT_NAME} ${WJWWOOD_SERIAL_UTESTS_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE wjwwood::serial)
target_link_libraries(${PROJECT_NAME} PRIVATE util)
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})