cmake_minimum_required(VERSION 3.7.2)
project(wjwwood_serial)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

include_directories(include)
set(WJWWOOD_SERIAL_SOURCES_BASE
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h)

if (WIN32)
set(WJWWOOD_SERIAL_SOURCES_PLATFORM
    include/serial/impl/unix.h
    src/impl/unix.cc
    src/impl/list_ports/list_ports_linux.cc)
elseif(UNIX)
    set(WJWWOOD_SERIAL_SOURCES_PLATFORM
        include/serial/impl/win.h
        src/impl/win.cc
        src/impl/list_ports/list_ports_win.cc)
elseif(APPLE)
    set(WJWWOOD_SERIAL_SOURCES_PLATFORM
        include/serial/impl/unix.h
        src/impl/unix.cc
        src/impl/list_ports/list_ports_osx.cc)
else()
    message(FATAL_ERROR "Platform not supported")
endif()

add_library(${PROJECT_NAME} ${WJWWOOD_SERIAL_SOURCES_BASE})
add_library(wjwwood::serial ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
target_include_directories(${PROJECT_NAME} PUBLIC include)