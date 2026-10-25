cmake_minimum_required(VERSION 3.16)
project(fiss_planner VERSION 1.1.0 LANGUAGES CXX)

option(FISS_PLANNER_BUILD_TESTS "Build unit tests" OFF)
option(FISS_PLANNER_BUILD_BENCHMARKS "Build micro-benchmarks" OFF)
option(FISS_PLANNER_SANITIZE "Build with AddressSanitizer and UBSan" OFF)
option(FISS_PLANNER_WITH_OSQP "Build the OSQP-based smoother and MPC controller if OSQP is found" ON)
option(FISS_PLANNER_VENDOR_DEPS
  "Always fetch and statically link yaml-cpp and OSQP, ignoring installed copies (self-contained Python packages)" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)
if(NOT FISS_PLANNER_VENDOR_DEPS)
  find_package(yaml-cpp QUIET)
endif()
if(NOT yaml-cpp_FOUND)
  # Self-contained builds (e.g. Python wheels): fetch and link yaml-cpp statically.
  message(STATUS "yaml-cpp not found, fetching it")
  include(FetchContent)
  FetchContent_Declare(yaml-cpp
    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
  set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
  FetchContent_MakeAvailable(yaml-cpp)
  set_target_properties(yaml-cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)
  set(FISS_PLANNER_FETCHED_YAML_CPP ON)
endif()

add_library(fiss_planner
  src/box2d.cpp
  src/frenet_obstacles.cpp
  src/lane_following.cpp
  src/components.cpp
  src/config.cpp
  src/controller.cpp
  src/mpc.cpp
  src/piecewise_jerk.cpp
  src/smoother.cpp
  src/corridor.cpp
  src/obstacle.cpp
  src/planner.cpp
  src/planners.cpp
  src/local_planner.cpp
  src/road_map.cpp
  src/route_corridor.cpp
  src/polynomial.cpp
  src/reference_line.cpp
  src/spline.cpp
)
add_library(fiss_planner::fiss_planner ALIAS fiss_planner)
target_compile_features(fiss_planner PUBLIC cxx_std_17)
set_target_properties(fiss_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fiss_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# yaml-cpp >= 0.8 exports yaml-cpp::yaml-cpp, older releases (Ubuntu 22.04) only yaml-cpp.
if(TARGET yaml-cpp::yaml-cpp)
  set(FISS_PLANNER_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(FISS_PLANNER_YAML_CPP_TARGET yaml-cpp)
endif()
target_link_libraries(fiss_planner PRIVATE ${FISS_PLANNER_YAML_CPP_TARGET} PUBLIC Threads::Threads)

set(FISS_PLANNER_HAS_OSQP OFF)
option(FISS_PLANNER_FETCH_OSQP "Fetch and statically link OSQP if it is not installed (wheels)" OFF)
if(FISS_PLANNER_WITH_OSQP)
  if(NOT FISS_PLANNER_VENDOR_DEPS)
    find_package(osqp QUIET)
  endif()
  if(NOT osqp_FOUND AND (FISS_PLANNER_FETCH_OSQP OR FISS_PLANNER_VENDOR_DEPS))
    message(STATUS "Fetching OSQP")
    include(FetchContent)
    set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
    set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
    set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
    set(OSQP_ENABLE_PROFILING OFF CACHE BOOL "" FORCE)
    set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(osqp
      URL https://github.com/osqp/osqp/archive/refs/tags/v1.0.0.tar.gz
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(osqp)
    set_target_properties(osqpstatic PROPERTIES POSITION_INDEPENDENT_CODE ON)
    add_library(osqp::osqpstatic ALIAS osqpstatic)
    set(osqp_FOUND TRUE)
    set(FISS_PLANNER_FETCHED_YAML_CPP ON)  # vendored deps: build-only, skip install/export
  endif()
  if(osqp_FOUND)
    if(TARGET osqp::osqpstatic AND NOT TARGET osqp::osqp)
      set(_osqp osqp::osqpstatic)
    else()
      set(_osqp osqp::osqp)
    endif()
    target_link_libraries(fiss_planner PRIVATE ${_osqp})
    target_compile_definitions(fiss_planner PRIVATE FISS_PLANNER_HAS_OSQP)
    set(FISS_PLANNER_HAS_OSQP ON)
  else()
    message(STATUS "OSQP not found: building without the QP smoother and MPC controller")
  endif()
endif()
message(STATUS "fiss_planner: OSQP optimizers ${FISS_PLANNER_HAS_OSQP}")
if(MSVC)
  target_compile_options(fiss_planner PRIVATE /W4)
else()
  target_compile_options(fiss_planner PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion
                                                -Wno-sign-conversion)
endif()

if(FISS_PLANNER_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
  target_compile_options(fiss_planner PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer
                                                -fno-sanitize-recover=all)
  target_link_options(fiss_planner PUBLIC -fsanitize=address,undefined)
endif()

# ----------------------------------------------------------------------------
# Tests and benchmarks
# ----------------------------------------------------------------------------
if(FISS_PLANNER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(FISS_PLANNER_BUILD_BENCHMARKS)
  add_subdirectory(bench)
endif()

# ----------------------------------------------------------------------------
# Install / export
# ----------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if(FISS_PLANNER_FETCHED_YAML_CPP)
  return()  # vendored dependency: build-only (Python wheel), no CMake package install
endif()

install(TARGETS fiss_planner EXPORT fiss_plannerTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT fiss_plannerTargets
  NAMESPACE fiss_planner::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fiss_planner
)
configure_package_config_file(cmake/fiss_plannerConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/fiss_plannerConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fiss_planner
)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/fiss_plannerConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/fiss_plannerConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/fiss_plannerConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fiss_planner
)
install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/fiss_planner)

