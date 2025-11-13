cmake_minimum_required(VERSION 3.15)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(
    Python
    3.11
    REQUIRED
    COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule
)

include(FetchContent)
FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind
    GIT_TAG v2.9.2
    # FIND_PACKAGE_ARGS
        # CONFIG # uses find_package first, git if it fails
)
FetchContent_MakeAvailable(nanobind)

nanobind_add_module(_core STABLE_ABI NB_STATIC cpp/main.cpp)
install(TARGETS _core DESTINATION ${SKBUILD_PROJECT_NAME})
