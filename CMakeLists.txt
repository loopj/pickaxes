# Set minimum CMake version
cmake_minimum_required(VERSION "3.21")

# Configure project and languages
project(libaxes LANGUAGES C)

# Define the library
add_library(axes INTERFACE)
target_sources(axes INTERFACE src/axes.c)

# Specify the include paths
target_include_directories(axes INTERFACE include)

# Build tests by default only when libaxes is the top-level project
option(AXES_BUILD_TESTS "Build axes tests" ${PROJECT_IS_TOP_LEVEL})

# Include tests if enabled
if(AXES_BUILD_TESTS)
  include(CTest)
  add_subdirectory(test)
endif()
