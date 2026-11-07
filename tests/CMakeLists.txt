add_executable(fusion_tests
  test_time.cpp
  test_sample_buffer.cpp
  test_imu.cpp
)
target_link_libraries(fusion_tests PRIVATE fusion::core GTest::gtest_main)
fusion_configure_target(fusion_tests)

include(GoogleTest)
gtest_discover_tests(fusion_tests)
