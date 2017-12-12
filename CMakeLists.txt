cmake_minimum_required(VERSION 3.7.2)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

enable_testing()
add_subdirectory(lib)
add_subdirectory(tests)
add_subdirectory(examples)