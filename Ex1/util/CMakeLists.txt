cmake_minimum_required(VERSION 3.7)
project(oskarVerifier)

# Use C++11 for this directory and its sub-directories.
list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_generalized_initializers has_cpp11)
if (has_cpp11 LESS 0)
  message(STATUS "NOTICE: These examples requires a C++11 compiler and will not be compiled.")
  return()
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS system thread filesystem program_options regex)

include_directories(${Boost_INCLUDE_DIR})
add_executable(oskarVerifier oskarVerifier.cpp)
target_link_libraries(oskarVerifier ${Boost_LIBRARIES})
