find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()
include(GoogleTest)

set(FISS_PLANNER_TESTS
  test_polynomial
  test_spline
  test_reference_line
  test_collision
  test_components
  test_config
  test_planners
  test_map
  test_control
  test_safety
  test_prediction
  test_behavior
  test_freespace
)

foreach(name ${FISS_PLANNER_TESTS})
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE fiss_planner GTest::gtest_main)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(${name} PRIVATE FISS_PLANNER_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../configs"
    FISS_PLANNER_PROFILE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../python/fiss_planner/profiles")
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endforeach()
