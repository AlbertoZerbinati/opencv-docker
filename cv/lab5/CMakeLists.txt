cmake_minimum_required(VERSION 2.8)
project(lab5)

find_package(OpenCV REQUIRED)

add_library(lab5 include/lab5.hpp src/lab5.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(task1.exe src/task1.cpp)
target_link_libraries(task1.exe ${OpenCV_LIBS} lab5)

# add_executable(task2.exe src/task2.cpp)
# target_link_libraries(task2.exe ${OpenCV_LIBS} lab5)

# add_executable(task3.exe src/task3.cpp)
# target_link_libraries(task3.exe ${OpenCV_LIBS} lab5)
