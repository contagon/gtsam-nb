# First thing, we check if vcpkg is installed in .vcpkg and if so, use it for dependencies
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/.vcpkg/scripts/buildsystems/vcpkg.cmake"
    )
endif()

# Can't use cmake > 4.0 b/c GTSAM doesn't support it yet
cmake_minimum_required(VERSION 3.15...3.31)
project(${SKBUILD_PROJECT_NAME} VERSION 0.0.1 LANGUAGES CXX) # x-release-please-version

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

# Make build a release build unless otherwise specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(
        CACHE CMAKE_BUILD_TYPE
        PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo"
    )
endif()

# ------------------------- Find Eigen / GTSAM ------------------------- #
find_package(Eigen3 NO_MODULE)

# Turn off extra building
set(GTSAM_BUILD_TESTS OFF CACHE BOOL "")
set(GTSAM_BUILD_DOCS OFF CACHE BOOL "")
set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "")
set(GTSAM_INSTALL_MATLAB_TOOLBOX OFF CACHE BOOL "")
set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "")
set(GTSAM_BUILD_EXAMPLES OFF CACHE BOOL "")
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "")

# use system eigen if found
if(Eigen3_FOUND)
    set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "")
endif()

# turn off deprecation warnings
add_compile_options(-Wno-deprecated-declarations)

FetchContent_Declare(
    GTSAM
    GIT_REPOSITORY https://github.com/borglab/gtsam.git
    GIT_TAG        4.2.0
    GIT_SHALLOW TRUE
    # GIT_PROGRESS   TRUE
    FIND_PACKAGE_ARGS # uses find_package first, git if it fails
)
FetchContent_MakeAvailable(GTSAM)

# ------------------------- Setup nanobind building ------------------------- #

find_package(
    Python
    3.11
    REQUIRED
    COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule
)

FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind
    GIT_TAG v2.9.2
    GIT_SHALLOW TRUE
    # GIT_PROGRESS   TRUE
    FIND_PACKAGE_ARGS CONFIG # uses find_package first, git if it fails
    # Should always be found though, provided via skbuild
)
FetchContent_MakeAvailable(nanobind)

nanobind_add_module(_core 
    STABLE_ABI NB_STATIC 
        cpp/main.cpp
        cpp/base.cpp
        cpp/inference.cpp
        cpp/discrete.cpp
        cpp/geometry.cpp
        cpp/linear.cpp
        cpp/nonlinear.cpp
        cpp/custom.cpp
        cpp/symbolic.cpp
        cpp/sam.cpp
        cpp/slam.cpp
        cpp/sfm.cpp
        cpp/navigation.cpp
        cpp/basis.cpp
        cpp/hybrid.cpp
)
target_link_libraries(_core PRIVATE Eigen3::Eigen gtsam)
install(TARGETS _core DESTINATION ${SKBUILD_PROJECT_NAME})

nanobind_add_stub(
  gtsam_stubs
  INSTALL_TIME
  MODULE gtsam._core
  RECURSIVE

  OUTPUT_PATH gtsam
  OUTPUT
      gtsam/__init__.pyi
      gtsam/submodule_1.pyi
      gtsam/submodule_2/__init__.pyi
      gtsam/submodule_2/nested.pyi
)