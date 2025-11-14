# Can't use cmake > 4.0 b/c GTSAM doesn't support it yet
cmake_minimum_required(VERSION 3.15...3.31)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

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

if(Eigen3_FOUND)
    set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "")
endif()

FetchContent_Declare(
    GTSAM
    GIT_REPOSITORY https://github.com/borglab/gtsam.git
    GIT_TAG        4.2.0
    GIT_SHALLOW TRUE
    # GIT_PROGRESS   TRUE
    # FIND_PACKAGE_ARGS # uses find_package first, git if it fails
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
    # FIND_PACKAGE_ARGS CONFIG # uses find_package first, git if it fails
    # Should always be found though, provided via skbuild
)
FetchContent_MakeAvailable(nanobind)

nanobind_add_module(_core 
    STABLE_ABI NB_STATIC 
        cpp/main.cpp
        cpp/base.cpp
        cpp/inference.cpp
        cpp/discrete.cpp
        # cpp/geometry.cpp
        # cpp/linear.cpp
        # cpp/nonlinear.cpp
        # cpp/custom.cpp
        # cpp/symbolic.cpp
        # cpp/sam.cpp
        # cpp/slam.cpp
        # cpp/sfm.cpp
        # cpp/navigation.cpp
        # cpp/basis.cpp
        # cpp/hybrid.cpp
)
target_link_libraries(_core PRIVATE Eigen3::Eigen gtsam)
install(TARGETS _core DESTINATION ${SKBUILD_PROJECT_NAME})

# TODO: Stubs