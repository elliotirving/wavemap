cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ros_conversions)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME}
    src/config_conversions.cc
    src/map_msg_conversions.cc
    src/time_conversions.cc)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen) # adds eigen to entire project

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_map_msg_conversions.cc)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main Eigen3::Eigen) # this adds eigen to TEST functions specifically
endif ()

# Export
cs_install()
cs_export()
