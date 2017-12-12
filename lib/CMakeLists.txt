project(wjwwood_serial)

if (APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif ()

include_directories(include)
set(WJWWOOD_SERIAL_SOURCES
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h)

if (WIN32)
    list(APPEND WJWWOOD_SERIAL_SOURCES
        include/serial/impl/win.h
        src/impl/win.cc
        src/impl/list_ports/list_ports_win.cc
        )
elseif (UNIX)
    list(APPEND WJWWOOD_SERIAL_SOURCES
        include/serial/impl/unix.h
        src/impl/unix.cc
        src/impl/list_ports/list_ports_linux.cc
        )
elseif (APPLE)
    list(APPEND WJWWOOD_SERIAL_SOURCES
        include/serial/impl/unix.h
        src/impl/unix.cc
        src/impl/list_ports/list_ports_osx.cc
        )
else ()
    message(FATAL_ERROR "Platform not supported")
endif ()

add_library(${PROJECT_NAME} ${WJWWOOD_SERIAL_SOURCES})
add_library(wjwwood::serial ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
target_include_directories(${PROJECT_NAME} PUBLIC include)