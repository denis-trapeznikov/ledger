#
# F E T C H   M A T H   E X A M P L E S
#
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(fetch-math)

# CMake configuration
include(${FETCH_ROOT_CMAKE_DIR}/BuildTools.cmake)

# Compiler Configuration
setup_compiler()

setup_library_examples(fetch-math)
