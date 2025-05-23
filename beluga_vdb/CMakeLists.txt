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

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

find_package(beluga REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(range-v3 REQUIRED)
find_package(Sophus REQUIRED)

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

add_library(${PROJECT_NAME} INTERFACE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(
    ${PROJECT_NAME}
    INTERFACE -Wall
              -Wconversion
              -Wextra
              -Werror
              -Wpedantic)
endif()
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  target_compile_options(${PROJECT_NAME} INTERFACE -fno-inline)
endif()

target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} INTERFACE beluga::beluga OpenVDB::openvdb)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_compile_definitions(${PROJECT_NAME} INTERFACE EIGEN_NO_DEBUG
                                                     SOPHUS_USE_BASIC_LOGGING)

add_executable(clang_tidy_findable)
target_sources(clang_tidy_findable PRIVATE src/clang_tidy_findable.cpp)
target_link_libraries(clang_tidy_findable PRIVATE ${PROJECT_NAME})

option(BUILD_TESTING "Build the testing tree." ON)
if(BUILD_TESTING)
  message(STATUS "Build testing enabled.")
  enable_testing()
  add_subdirectory(test)
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

set(INSTALL_CMAKEDIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${INSTALL_CMAKEDIR})

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${INSTALL_CMAKEDIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${INSTALL_CMAKEDIR})
