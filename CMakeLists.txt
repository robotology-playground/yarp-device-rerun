# Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.16...3.27)
project(example LANGUAGES CXX)

# add_executable(example_minimal main.cpp)


include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

find_package(YARP 3.12.0 REQUIRED)
add_executable(example_rerun)
target_sources(example_rerun PRIVATE main.cpp Example.cpp Example.h)

target_link_libraries(example_rerun  PRIVATE  YARP::YARP_os
                                        YARP::YARP_dev
                                        YARP::YARP_init
                                        rerun_sdk)

install(
  TARGETS example_rerun
  COMPONENT utilities
  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

set_property(TARGET example_rerun PROPERTY FOLDER "Command Line Tools")
# yarp_prepare_plugin(example
#                     CATEGORY device
#                     TYPE Example
#                     INCLUDE Example.h
#                     DEFAULT ON)
#                     # GENERATE_PARSER)

# if(ENABLE_example)
#     yarp_add_plugin(YARP_example)

#     target_sources(YARP_example  PRIVATE Example.cpp
#                                     Example.h
#                                     main.cpp
#                                                         # Example_ParamsParser.cpp
#                                                         # Example_ParamsParser.h
#                                                         )

#     target_link_libraries(YARP_example  PRIVATE  YARP::YARP_os
#                                             YARP::YARP_dev
#                                             YARP::YARP_init
#                                             rerun_sdk)
#     list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
#                                                         YARP_dev
#                                                         YARP::YARP_init)

#     yarp_install(TARGETS YARP_example
#                 EXPORT YARP_${YARP_PLUGIN_MASTER}
#                 COMPONENT ${YARP_PLUGIN_MASTER}
#                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
#                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
#                 YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

#     set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

#     set_property(TARGET YARP_example PROPERTY FOLDER "Plugins/Device")
# endif()