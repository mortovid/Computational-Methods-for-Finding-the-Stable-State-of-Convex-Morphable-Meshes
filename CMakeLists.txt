cmake_minimum_required(VERSION 3.6)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project Name
project(CCGHomework)

# Silence OpenGL Deprecation Warnings on macOS
add_definitions(-DGL_SILENCE_DEPRECATION)

# Ensure compatibility with Apple Silicon (M1/M2/M3) and Intel Macs
if(APPLE)
  set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64")
endif()

# Include Eigen Library
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/eigen-3.4.0")
set(MESHLIB_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/MeshLib/core")
include_directories(${EIGEN3_INCLUDE_DIR} ${MESHLIB_INCLUDE_DIR})

# Find OpenGL and GLUT
if(APPLE)
  find_package(OpenGL REQUIRED)
  find_library(GLUT_LIBRARY NAMES glut PATHS /opt/homebrew/lib /usr/local/lib)
  include_directories(/opt/homebrew/include /usr/local/include)
  link_directories(/opt/homebrew/lib /usr/local/lib)
else()
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
endif()

# Define Executable and Source Files
include_directories(${CMAKE_SOURCE_DIR}/cutgraph/include)
file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/cutgraph/src/*.cpp")
add_executable(CCGHomework ${SRC_FILES})

# **Properly Link OpenGL and GLUT**
if(APPLE)
  target_link_libraries(CCGHomework
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
  )
else()
  target_link_libraries(CCGHomework ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
endif()

# Add the cutgraph module
add_subdirectory("cutgraph")
