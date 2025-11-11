# Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.16...3.27)
project(yarp-device-rerun LANGUAGES CXX)

find_package(YARP 3.12.0 REQUIRED)
find_package(YCM REQUIRED)
find_package(iDynTree REQUIRED COMPONENTS idyntree-model idyntree-modelio idyntree-kinematics)

# Support either rerun from find_package, or fallback to FetchContent
# By default, set YARP_DEVICE_RERUN_USE_SYSTEM_RERUN to ON if find_package(rerun_sdk) works
set(YARP_DEVICE_RERUN_USE_SYSTEM_RERUN_DEFAULT_VALUE "OFF")
if(NOT DEFINED YARP_DEVICE_RERUN_USE_SYSTEM_RERUN)
  find_package(rerun_sdk QUIET)
  set(YARP_DEVICE_RERUN_USE_SYSTEM_RERUN_DEFAULT_VALUE ${rerun_sdk_FOUND})
endif()

option(YARP_DEVICE_RERUN_USE_SYSTEM_RERUN "If ON, find rerun_sdk with find_package" ${YARP_DEVICE_RERUN_USE_SYSTEM_RERUN_DEFAULT_VALUE})

if(YARP_DEVICE_RERUN_USE_SYSTEM_RERUN)
  find_package(rerun_sdk REQUIRED)
else()
  include(FetchContent)
  FetchContent_Declare(rerun_sdk URL
      https://github.com/rerun-io/rerun/releases/download/0.24.1/rerun_cpp_sdk.zip)
  FetchContent_MakeAvailable(rerun_sdk)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "yarp-device-rerun is always built with dynamic plugins")
set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "Build libraries as shared as opposed to static")

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS 1)

add_subdirectory(src)
include(AddUninstallTarget)
