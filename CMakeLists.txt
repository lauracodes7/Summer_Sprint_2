
project(sprint_2)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(mission_planner
  src/distance_cost_estimator.cpp
  src/main.cpp
  src/navigator_2d.cpp
)


# ---===== Fetch GoogleTest =====---
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.8.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


# ---===== Tests =====---
set(CTEST_OUTPUT_ON_FAILURE ON)

enable_testing()

add_executable(test_navigator_2d tests/test_navigator_2d.cpp src/navigator_2d.cpp)
target_link_libraries(test_navigator_2d gtest gtest_main)
add_test(NAME test_navigator_2d COMMAND test_navigator_2d)
