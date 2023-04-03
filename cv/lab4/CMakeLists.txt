cmake_minimum_required(VERSION 2.8)
project(lab4)

find_package(OpenCV REQUIRED)

add_library(lab4 include/lab4.hpp src/lab4.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(task1.exe src/task1.cpp)
target_link_libraries(task1.exe ${OpenCV_LIBS} lab4)

add_executable(task2.exe src/task2.cpp)
target_link_libraries(task2.exe ${OpenCV_LIBS} lab4)

# add_executable(task3.exe src/task3.cpp)
# target_link_libraries(task3.exe ${OpenCV_LIBS})

# add_executable(task4.exe src/task4.cpp)
# target_link_libraries(task4.exe ${OpenCV_LIBS})
