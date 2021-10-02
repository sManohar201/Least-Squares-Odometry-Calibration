cmake_minimum_required(VERSION 3.9)

project(least_squares_calibration)

set(CMAKE_CXX_STANDARD 11)

# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS
              program_options)
find_package(Eigen3 REQUIRED)

include_directories(
              include
              ${PYTHON_INCLUDE_DIRS}
              ${Boost_INCLUDE_DIRS}
)
add_library(file_reader src/read_files.cpp)


set(SOURCE src/main.cpp
           src/least_squares.cpp
           include/pbplot/pbPlots.cpp 
           include/pbplot/supportLib.cpp)   


add_executable(${PROJECT_NAME} ${SOURCE})

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} file_reader ${PYTHOND_LIBRARIES} Boost::program_options)

