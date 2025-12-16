# Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

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

    set(yarpLoggerRerun_LINK_LIBRARIES
        YARP::YARP_os
        YARP::YARP_dev
        YARP::YARP_init
        iDynTree::idyntree-model
        iDynTree::idyntree-modelio
        iDynTree::idyntree-high-level
        rerun_sdk
    )

    if(YARP_DEVICE_RERUN_ENABLE_RAWVALUES)
        list(APPEND yarpLoggerRerun_LINK_LIBRARIES ICUB::iCubDev)
        target_compile_definitions(yarpLoggerRerun PUBLIC -DUSE_RAWVALUES_PUBLISHER)
    endif()

    target_link_libraries(yarpLoggerRerun PUBLIC ${yarpLoggerRerun_LINK_LIBRARIES})

    yarp_install(TARGETS yarpLoggerRerun
                EXPORT yarp-device-rerun
                COMPONENT yarp-device-rerun
                LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
                YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

    set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

    set_property(TARGET yarpLoggerRerun PROPERTY FOLDER "Plugins/Device")
endif()