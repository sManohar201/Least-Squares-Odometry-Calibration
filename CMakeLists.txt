cmake_minimum_required(VERSION 3.5)

project(least_squares_calibration)

set(CMAKE_CXX_STANDARD 14)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS
              program_options)
find_package(Eigen3 REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

set(SOURCE src/main.cpp
            src/least_squares.cpp)   


add_executable(${PROJECT_NAME} ${SOURCE})
target_link_directories(${PROJECT_NAME} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PYTHOND_LIBRARIES} Boost::program_options)

