# Copyright 2025 Ekumen, Inc.
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

################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.10)
project(beluga_rviz_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wnon-virtual-dtor
    -Woverloaded-virtual)
endif()

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

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rviz_common
    rviz_default_plugins
    rviz_rendering
    rviz_ogre_vendor
    pluginlib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenVDB REQUIRED)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.
set(BELUGA_RVIZ_PLUGINS_HEADERS_TO_MOC
    include/beluga_rviz_plugins/vdbmap_display.hpp)

foreach(header "${BELUGA_RVIZ_PLUGINS_HEADERS_TO_MOC}")
  qt5_wrap_cpp(beluga_rviz_plugins_moc_files "${header}")
endforeach()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

################################################################################
# Build
################################################################################
# Plugin library
add_library(${PROJECT_NAME} SHARED ${beluga_rviz_plugins_moc_files}
                                   src/vdbmap_display.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC ${Qt5Widgets_INCLUDE_DIRS}
         ${OGRE_INCLUDE_DIRS}
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rviz_common::rviz_common
         rviz_default_plugins::rviz_default_plugins
         rviz_rendering::rviz_rendering
         OpenVDB::openvdb)
target_link_libraries(${PROJECT_NAME} PRIVATE ament_index_cpp::ament_index_cpp
                                              pluginlib::pluginlib)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

################################################################################
# Install
################################################################################
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rviz_common rviz_default_plugins rviz_rendering)
ament_package()
