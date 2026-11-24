# ---------------------------------------------------------------------------
# FusionEKF: an INS / GNSS / wheel-odometry / vision error-state EKF.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.25)
project(FusionEKF VERSION 0.1.0 LANGUAGES CXX)

# Language standard is a project-wide policy, so it is fine to set globally.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)          # -std=c++23, not -std=gnu++23
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # for clangd / clang-tidy

option(FUSION_BUILD_TESTS "Build the unit tests" ON)
option(FUSION_ENABLE_SANITIZERS "Build with AddressSanitizer + UBSan" OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(ProjectOptions)   # fusion_configure_target()
include(Dependencies)     # Eigen3::Eigen, GTest::gtest_main

# ---- Core library ---------------------------------------------------------
add_library(fusion_core
  src/filter/kalman_filter.cpp
  src/models/constant_velocity_2d.cpp
  src/sim/circle_simulator.cpp
  src/io/results_csv.cpp
)
add_library(fusion::core ALIAS fusion_core)   # consumers use the namespaced name

target_include_directories(fusion_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(fusion_core PUBLIC Eigen3::Eigen)
target_compile_features(fusion_core PUBLIC cxx_std_23)
fusion_configure_target(fusion_core)

# ---- Demo programs --------------------------------------------------------
add_executable(car_1d_demo apps/car_1d_demo.cpp)
target_link_libraries(car_1d_demo PRIVATE fusion::core)
fusion_configure_target(car_1d_demo)
set_target_properties(car_1d_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps)

add_executable(car_2d_demo apps/car_2d_demo.cpp)
target_link_libraries(car_2d_demo PRIVATE fusion::core)
fusion_configure_target(car_2d_demo)
set_target_properties(car_2d_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps)

# ---- Tests ----------------------------------------------------------------
if(FUSION_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
