# Copyright 2024 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

project(beluga_vdb VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OPENVDB_CMAKE_MODULE_PATH
    ${OPENVDB_CMAKE_MODULE_PATH}
    CACHE PATH "Path to OpenVDB CMake module")

if(NOT OPENVDB_CMAKE_MODULE_PATH)
  file(
    GLOB_RECURSE
    OPENVDB_MODULES
    /usr/lib/*/FindOpenVDB.cmake
    /usr/local/lib/*/FindOpenVDB.cmake)
  list(LENGTH OPENVDB_MODULES NUM_OPENVDB_MODULES)
  if(NUM_OPENVDB_MODULES EQUAL 1)
    list(
      GET
      OPENVDB_MODULES
      0
      OPENVDB_MODULE)
    get_filename_component(OPENVDB_CMAKE_MODULE_PATH ${OPENVDB_MODULE}
                           DIRECTORY)
  endif()
  unset(NUM_OPENVDB_MODULES)
  unset(OPENVDB_MODULES)
endif()

if(OPENVDB_CMAKE_MODULE_PATH)
  list(APPEND CMAKE_MODULE_PATH ${OPENVDB_CMAKE_MODULE_PATH})
endif()

find_package(OpenVDB REQUIRED)

if(OPENVDB_CMAKE_MODULE_PATH)
  list(POP_BACK CMAKE_MODULE_PATH)
endif()

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wconversion
    -Wextra
    -Werror
    -Wpedantic)
endif()
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-fno-inline)
endif()

set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_VERSION $ENV{ROS_VERSION})
if(NOT ROS_VERSION)
  if(CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)
    set(ROS_VERSION 1)
  else()
    find_package(ament_cmake QUIET)
    if(ament_cmake_FOUND)
      set(ROS_VERSION 2)
    endif()
  endif()
endif()
if(ROS_VERSION EQUAL 2)
  include(cmake/ament.cmake)
elseif(ROS_VERSION EQUAL 1)
  include(cmake/catkin.cmake)
else()
  message(FATAL_ERROR "Unknown ROS distribution version: ${ROS_VERSION}")
endif()
