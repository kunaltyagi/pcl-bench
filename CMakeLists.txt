# Following line is required to perform compatibility checks
# Make sure that the version is at least 3.5, as required by PCL
cmake_minimum_required(VERSION 3.5)
project(pcl_bench)

# Use the following to link against all the PCL libraries
# Note: might be heavy, not the best practice
# find_package(PCL)
# Prefer to use the correct component libraries instead
find_package(PCL COMPONENTS common)

find_package(benchmark REQUIRED)

# All header directories not under your control go here
# ${PCL_INCLUDE_DIRS} not required to be manually added since PCL 1.9
include_directories(SYSTEM ${benchmark_INCLUDE_DIRS})
# Headers for this project go here
# include_directories(include)

# Add a target to compile for cmake
# CMake automatically chooses the correct flags for executable or library
add_executable(pcl_bench bench.cpp)
# add_library(pcl_demo main.cpp)

# Add the required libraries
target_link_libraries(pcl_bench ${PCL_LIBRARIES} benchmark::benchmark)
# Add the dependencies, if any
# add_dependencies(pcl_demo some_other_target)
