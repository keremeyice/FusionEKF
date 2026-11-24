add_executable(fusion_tests
  test_kalman_filter.cpp
  test_constant_velocity_2d.cpp
  test_circle_simulator.cpp
  test_results_csv.cpp
)
target_link_libraries(fusion_tests PRIVATE fusion::core GTest::gtest_main)
fusion_configure_target(fusion_tests)

include(GoogleTest)
gtest_discover_tests(fusion_tests)
