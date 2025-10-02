# Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/download/0.24.1/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

yarp_prepare_plugin(yarpLoggerRerun
                    CATEGORY device
                    TYPE YarpLoggerRerun
                    INCLUDE YarpLoggerRerun.h
                    DEFAULT ON
                    GENERATE_PARSER)

if(ENABLE_yarpLoggerRerun)
    yarp_add_plugin(yarpLoggerRerun)

    target_sources(yarpLoggerRerun PRIVATE  YarpLoggerRerun.cpp
                                            YarpLoggerRerun.h
                                            YarpLoggerRerun_ParamsParser.cpp
                                            YarpLoggerRerun_ParamsParser.h)

    target_link_libraries(yarpLoggerRerun PUBLIC  YARP::YARP_os
                                                        YARP::YARP_dev
                                                        YARP::YARP_init
                                                        rerun_sdk
                                                        iDynTree::idyntree-model
                                                        iDynTree::idyntree-modelio
                                                        iDynTree::idyntree-high-level)

    yarp_install(TARGETS yarpLoggerRerun
                EXPORT yarp-device-rerun
                COMPONENT yarp-device-rerun
                LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
                YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

    set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

    set_property(TARGET yarpLoggerRerun PROPERTY FOLDER "Plugins/Device")
endif()