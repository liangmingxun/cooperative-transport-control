cmake_minimum_required(VERSION 3.16)
project(cooperative_transport_control VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PAPER_BUILD_GEOMETRY "Build the path, geometry and planar execution modules (Eigen3)" ON)
find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)
file(GLOB_RECURSE PAPER_CONFIG_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/config/*.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PAPER_CONFIG_FILES} tools/generate_parameters.py)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_parameters.py"
          --config "${CMAKE_CURRENT_SOURCE_DIR}/config"
          --output "${CMAKE_CURRENT_BINARY_DIR}/generated/paper_control/parameters.hpp"
  RESULT_VARIABLE parameter_result)
if(NOT parameter_result EQUAL 0)
  message(FATAL_ERROR "Parameter generation failed")
endif()

add_library(transport_control
  src/dynamic_boundary.cpp src/upper_reference_generator.cpp
  src/lower_channel_controller.cpp src/m2b_controller.cpp
  src/capability_mapper.cpp src/derating_profile.cpp
  src/wheel_command_limiter.cpp src/margin_feedback.cpp)
target_include_directories(transport_control PUBLIC include "${CMAKE_CURRENT_BINARY_DIR}/generated")
if(MSVC)
  target_compile_options(transport_control PRIVATE /W4)
else()
  target_compile_options(transport_control PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(PAPER_BUILD_GEOMETRY)
  find_package(Eigen3 3.3 QUIET NO_MODULE)
  if(NOT TARGET Eigen3::Eigen)
    find_path(EIGEN3_INCLUDE_DIR Eigen/Core PATH_SUFFIXES eigen3)
    if(NOT EIGEN3_INCLUDE_DIR)
      message(FATAL_ERROR "Eigen3 headers required. Set EIGEN3_INCLUDE_DIR or use -DPAPER_BUILD_GEOMETRY=OFF for core-only examples.")
    endif()
    add_library(paper_eigen INTERFACE)
    target_include_directories(paper_eigen SYSTEM INTERFACE "${EIGEN3_INCLUDE_DIR}")
    add_library(Eigen3::Eigen ALIAS paper_eigen)
  endif()
  target_sources(transport_control PRIVATE src/s_curve_path.cpp src/support_geometry.cpp
    src/path_projector.cpp src/state_estimator.cpp src/planar_support_tracker.cpp
    src/nominal_speed_baseline.cpp)
  target_link_libraries(transport_control PUBLIC Eigen3::Eigen)
  target_compile_definitions(transport_control PUBLIC PAPER_WITH_GEOMETRY=1)
endif()

add_executable(algorithm_example examples/algorithm_example.cpp)
target_link_libraries(algorithm_example PRIVATE transport_control)
add_executable(scenario_example examples/scenario_example.cpp)
target_link_libraries(scenario_example PRIVATE transport_control)
