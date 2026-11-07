# Third-party dependencies, downloaded at configure time with FetchContent.
include(FetchContent)

# ---- Eigen (header-only linear algebra; the de-facto standard in robotics) --
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    # Eigen is header-only; point SOURCE_SUBDIR at a non-existent folder so
    # FetchContent only downloads it instead of running Eigen's big CMake build.
    SOURCE_SUBDIR do-not-build
  )
  FetchContent_MakeAvailable(eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  # IMPORTED targets' include dirs are treated as SYSTEM -> no warnings from Eigen.
  target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

# ---- GoogleTest -------------------------------------------------------------
if(FUSION_BUILD_TESTS)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    SYSTEM
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()
