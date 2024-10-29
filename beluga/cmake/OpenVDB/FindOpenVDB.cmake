# Copyright Contributors to the OpenVDB Project
# SPDX-License-Identifier: MPL-2.0
#
#[=======================================================================[.rst:

FindOpenVDB
-----------

Find OpenVDB include dirs, libraries and settings

Use this module by invoking find_package with the form::

  find_package(OpenVDB
    [version] [EXACT]      # Minimum or EXACT version
    [REQUIRED]             # Fail with error if OpenVDB is not found
    [COMPONENTS <libs>...] # OpenVDB libraries by their canonical name
                           # e.g. "openvdb" for "libopenvdb"
    )

IMPORTED Targets
^^^^^^^^^^^^^^^^

``OpenVDB::openvdb``
  The core openvdb library target.


Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``OpenVDB_FOUND``
  True if the system has the OpenVDB library.
``OpenVDB_VERSION``
  The version of the OpenVDB library which was found.
``OpenVDB_INCLUDE_DIRS``
  Include directories needed to use OpenVDB.
``OpenVDB_LIBRARIES``
  Libraries needed to link to OpenVDB.
``OpenVDB_LIBRARY_DIRS``
  OpenVDB library directories.
``OpenVDB_DEFINITIONS``
  Definitions to use when compiling code that uses OpenVDB.
``OpenVDB_${COMPONENT}_FOUND``
  True if the system has the named OpenVDB component.


Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``OpenVDB_INCLUDE_DIR``
  The directory containing ``openvdb/version.h``.


Hints
^^^^^

Instead of explicitly setting the cache variables, the following variables
may be provided to tell this module where to look.

``OpenVDB_ROOT``
  Preferred installation prefix.
``OPENVDB_INCLUDEDIR``
  Preferred include directory e.g. <prefix>/include
``OPENVDB_LIBRARYDIR``
  Preferred library directory e.g. <prefix>/lib
``SYSTEM_LIBRARY_PATHS``
  Global list of library paths intended to be searched by and find_xxx call
``OPENVDB_USE_STATIC_LIBS``
  Only search for static openvdb libraries
``DISABLE_CMAKE_SEARCH_PATHS``
  Disable CMakes default search paths for find_xxx calls in this module

#]=======================================================================]

cmake_minimum_required(VERSION 3.12)
include(GNUInstallDirs)

# Include utility functions for version information
include(${CMAKE_CURRENT_LIST_DIR}/OpenVDBUtils.cmake)

mark_as_advanced(
  OpenVDB_INCLUDE_DIR
  OpenVDB_LIBRARY
)

set(_FIND_OPENVDB_ADDITIONAL_OPTIONS "")
if(DISABLE_CMAKE_SEARCH_PATHS)
  set(_FIND_OPENVDB_ADDITIONAL_OPTIONS NO_DEFAULT_PATH)
endif()

# Core OpenVDB library only
list(INSERT OpenVDB_FIND_COMPONENTS 0 openvdb)

# Locate OpenVDB root directory, if provided
# Set _OPENVDB_ROOT based on a user provided root var. Xxx_ROOT and ENV{Xxx_ROOT}
# are prioritised over the legacy capitalized XXX_ROOT variables for matching
# CMake 3.12 behaviour
# @todo  deprecate -D and ENV OPENVDB_ROOT from CMake 3.12
if(OpenVDB_ROOT)
  set(_OPENVDB_ROOT ${OpenVDB_ROOT})
elseif(DEFINED ENV{OpenVDB_ROOT})
  set(_OPENVDB_ROOT $ENV{OpenVDB_ROOT})
elseif(OPENVDB_ROOT)
  set(_OPENVDB_ROOT ${OPENVDB_ROOT})
elseif(DEFINED ENV{OPENVDB_ROOT})
  set(_OPENVDB_ROOT $ENV{OPENVDB_ROOT})
endif()

# Additionally try and use pkconfig to find OpenVDB
if(USE_PKGCONFIG)
  if(NOT DEFINED PKG_CONFIG_FOUND)
    find_package(PkgConfig)
  endif()
  pkg_check_modules(PC_OpenVDB QUIET OpenVDB)
endif()

# This CMake module supports being called from external packages AND from
# within the OpenVDB repository for building openvdb components with the
# core library build disabled. Determine where we are being called from:
#
# (repo structure = <root>/cmake/FindOpenVDB.cmake)
# (inst structure = <root>/lib/cmake/OpenVDB/FindOpenVDB.cmake)

get_filename_component(_DIR_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

if(${_DIR_NAME} STREQUAL "cmake")
  # Called from root repo for openvdb components
elseif(${_DIR_NAME} STREQUAL "OpenVDB")
  # Set the install variable to track directories if this is being called from
  # an installed location and from another package. The expected installation
  # directory structure is:
  #  <root>/lib/cmake/OpenVDB/FindOpenVDB.cmake
  #  <root>/include
  #  <root>/bin
  get_filename_component(_IMPORT_PREFIX ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
  get_filename_component(_IMPORT_PREFIX ${_IMPORT_PREFIX} DIRECTORY)
  get_filename_component(_IMPORT_PREFIX ${_IMPORT_PREFIX} DIRECTORY)
  set(_OPENVDB_INSTALL ${_IMPORT_PREFIX})
  list(APPEND _OPENVDB_ROOT ${_OPENVDB_INSTALL})
endif()

unset(_DIR_NAME)
unset(_IMPORT_PREFIX)

# ------------------------------------------------------------------------
#  Search for OpenVDB include DIR
# ------------------------------------------------------------------------

# Clear previous paths to avoid conflicts
set(_OPENVDB_INCLUDE_SEARCH_DIRS "")

# Define default search paths for OpenVDB installation from binaries
list(APPEND _OPENVDB_INCLUDE_SEARCH_DIRS
  # This should be the primary include directory for OpenVDB (usually includes "openvdb/openvdb.h")
  "/usr/local/include/openvdb"                 # macOS or custom install paths
  "/usr/include/openvdb"                       # Common Linux install path
  "/usr/local/opt/openvdb/include/openvdb"     # Homebrew or other package managers

  # Optional custom include and root directories if known
  ${OPENVDB_INCLUDEDIR}                        # Custom include directory if set
  ${_OPENVDB_ROOT}                             # Root directory, if OpenVDB is within a package
  ${PC_OpenVDB_INCLUDE_DIRS}                   # Any OpenVDB package-configured include directory
  ${SYSTEM_LIBRARY_PATHS}                      # Additional system library paths
)

# Define the library search paths, ensuring OpenVDB library path is accessible
list(APPEND _OPENVDB_LIBRARY_SEARCH_DIRS
  "/usr/local/lib"                             # macOS or custom library paths
  "/usr/lib"                                   # Common Linux library path
  "/usr/local/opt/openvdb/lib"                 # Homebrew or other package managers
)

set(_OPENVDB_INCLUDE_SEARCH_DIRS ${OPENVDB_INCLUDEDIR} ${_OPENVDB_ROOT} ${SYSTEM_LIBRARY_PATHS})
find_path(OpenVDB_openvdb_INCLUDE_DIR openvdb/version.h
  ${_FIND_OPENVDB_ADDITIONAL_OPTIONS}
  PATHS ${_OPENVDB_INCLUDE_SEARCH_DIRS}
  PATH_SUFFIXES ${CMAKE_INSTALL_INCLUDEDIR} include
)

set(OpenVDB_INCLUDE_DIR ${OpenVDB_openvdb_INCLUDE_DIR} CACHE PATH "The OpenVDB core include directory")

# Retrieve OpenVDB version from header
set(_OPENVDB_VERSION_HEADER "${OpenVDB_INCLUDE_DIR}/openvdb/version.h")
OPENVDB_VERSION_FROM_HEADER("${_OPENVDB_VERSION_HEADER}"
  VERSION OpenVDB_VERSION
  MAJOR   OpenVDB_MAJOR_VERSION
  MINOR   OpenVDB_MINOR_VERSION
  PATCH   OpenVDB_PATCH_VERSION
  ABI     OpenVDB_ABI_FROM_HEADER # will be OpenVDB_MAJOR_VERSION prior to 8.1.0
)

if(OpenVDB_VERSION VERSION_LESS 8.1.0)
  set(_OPENVDB_HAS_NEW_VERSION_HEADER FALSE)
  # ABI gets computed later
else()
  set(_OPENVDB_HAS_NEW_VERSION_HEADER TRUE)
  set(OpenVDB_ABI ${OpenVDB_ABI_FROM_HEADER})
endif()
unset(OpenVDB_ABI_FROM_HEADER)

# Search for OpenVDB library directory
set(_OPENVDB_LIBRARYDIR_SEARCH_DIRS ${OPENVDB_LIBRARYDIR} ${_OPENVDB_ROOT} ${SYSTEM_LIBRARY_PATHS})
find_library(OpenVDB_openvdb_LIBRARY openvdb
  ${_FIND_OPENVDB_ADDITIONAL_OPTIONS}
  PATHS ${_OPENVDB_LIBRARYDIR_SEARCH_DIRS}
  PATH_SUFFIXES ${CMAKE_INSTALL_LIBDIR} lib64 lib
)
set(OpenVDB_LIBRARIES ${OpenVDB_openvdb_LIBRARY} CACHE STRING "The OpenVDB core library")


# ------------------------------------------------------------------------
#  Search for OPENVDB lib DIR
# ------------------------------------------------------------------------

set(_OPENVDB_LIBRARYDIR_SEARCH_DIRS "")

# Append to _OPENVDB_LIBRARYDIR_SEARCH_DIRS in priority order
list(APPEND _OPENVDB_LIBRARYDIR_SEARCH_DIRS
  ${OPENVDB_LIBRARYDIR}
  ${_OPENVDB_ROOT}
  ${PC_OpenVDB_LIBRARY_DIRS}
  ${SYSTEM_LIBRARY_PATHS}
)

# Library suffix handling
set(_OPENVDB_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})

set(OPENVDB_PYTHON_PATH_SUFFIXES
  ${CMAKE_INSTALL_LIBDIR}/python
  ${CMAKE_INSTALL_LIBDIR}/python2.7
  ${CMAKE_INSTALL_LIBDIR}/python3
  lib64/python
  lib64/python2.7
  lib64/python3
  lib/python
  lib/python2.7
  lib/python3
)

set(OPENVDB_LIB_PATH_SUFFIXES
  ${CMAKE_INSTALL_LIBDIR}
  lib64
  lib
)

list(REMOVE_DUPLICATES OPENVDB_PYTHON_PATH_SUFFIXES)
list(REMOVE_DUPLICATES OPENVDB_LIB_PATH_SUFFIXES)

# Static library setup
if(WIN32)
  if(OPENVDB_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  endif()
else()
  if(OPENVDB_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
  endif()
endif()

unset(OPENVDB_PYTHON_PATH_SUFFIXES)
unset(OPENVDB_LIB_PATH_SUFFIXES)

# Reset library suffix
set(CMAKE_FIND_LIBRARY_SUFFIXES ${_OPENVDB_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
unset(_OPENVDB_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES)


# ------------------------------------------------------------------------
#  Cache and set OPENVDB_FOUND
# ------------------------------------------------------------------------

# Set OpenVDB_FOUND to true if the core library was found
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenVDB
  FOUND_VAR OpenVDB_FOUND
  REQUIRED_VARS
    OpenVDB_INCLUDE_DIR
  VERSION_VAR OpenVDB_VERSION
  HANDLE_COMPONENTS
)


# ------------------------------------------------------------------------
#  Determine ABI number
# ------------------------------------------------------------------------

# Set the ABI number the library was built against. The old system,
# which didn't define the ABI in the build config, uses vdb_print

if(NOT _OPENVDB_HAS_NEW_VERSION_HEADER)
  if(_OPENVDB_INSTALL)
    OPENVDB_ABI_VERSION_FROM_PRINT(
      "${_OPENVDB_INSTALL}/bin/vdb_print"
      ABI OpenVDB_ABI
    )
  else()
    # Try and find vdb_print from the include path
    OPENVDB_ABI_VERSION_FROM_PRINT(
      "${OpenVDB_INCLUDE_DIR}/../bin/vdb_print"
      ABI OpenVDB_ABI
    )
  endif()
endif()

if(NOT OpenVDB_FIND_QUIET)
  if(NOT OpenVDB_ABI)
    message(WARNING "Unable to determine OpenVDB ABI version from OpenVDB "
      "installation. The library major version \"${OpenVDB_MAJOR_VERSION}\" "
      "will be inferred. If this is not correct, use "
      "add_definitions(-DOPENVDB_ABI_VERSION_NUMBER=N)"
    )
  else()
    message(STATUS "OpenVDB ABI Version: ${OpenVDB_ABI}")
  endif()
endif()


# ------------------------------------------------------------------------
#  Configure imported targets
# ------------------------------------------------------------------------

set(OpenVDB_INCLUDE_DIRS ${OpenVDB_INCLUDE_DIR})

set(OpenVDB_LIBRARY_DIRS "")
get_filename_component(_OPENVDB_LIBDIR openvdb DIRECTORY)
list(APPEND OpenVDB_LIBRARY_DIRS ${_OPENVDB_LIBDIR})

# OpenVDB::openvdb

if(NOT TARGET OpenVDB::openvdb)
  set(OPENVDB_openvdb_LIB_TYPE UNKNOWN)
  if(OPENVDB_USE_STATIC_LIBS)
    set(OPENVDB_openvdb_LIB_TYPE STATIC)
  elseif(UNIX)
    get_filename_component(_OPENVDB_openvdb_EXT
      ${OpenVDB_openvdb_LIBRARY} EXT)
    if(_OPENVDB_openvdb_EXT STREQUAL ".a")
      set(OPENVDB_openvdb_LIB_TYPE STATIC)
    elseif(_OPENVDB_openvdb_EXT STREQUAL ".so" OR
           _OPENVDB_openvdb_EXT STREQUAL ".dylib")
      set(OPENVDB_openvdb_LIB_TYPE SHARED)
    endif()
  endif()

  add_library(OpenVDB::openvdb ${OPENVDB_openvdb_LIB_TYPE} IMPORTED)
  set_target_properties(OpenVDB::openvdb PROPERTIES
    IMPORTED_LOCATION "${OpenVDB_openvdb_LIBRARY}"
    INTERFACE_COMPILE_OPTIONS "${PC_OpenVDB_CFLAGS_OTHER}"
    INTERFACE_COMPILE_DEFINITIONS "${OpenVDB_DEFINITIONS}"
    INTERFACE_INCLUDE_DIRECTORIES "${OpenVDB_INCLUDE_DIR}"
    IMPORTED_LINK_DEPENDENT_LIBRARIES "${_OPENVDB_HIDDEN_DEPENDENCIES}" # non visible deps
    INTERFACE_LINK_LIBRARIES "${_OPENVDB_VISIBLE_DEPENDENCIES}" # visible deps (headers)
    INTERFACE_COMPILE_FEATURES cxx_std_14
  )
endif()

unset(_OPENVDB_VISIBLE_DEPENDENCIES)
unset(_OPENVDB_HIDDEN_DEPENDENCIES)