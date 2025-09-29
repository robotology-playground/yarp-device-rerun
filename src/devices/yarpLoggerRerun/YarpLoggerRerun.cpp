/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IAxisInfo.h>
#include <rerun/log_sink.hpp>

#include "YarpLoggerRerun.h"

#include <stdlib.h>
#include <ctime>

YARP_LOG_COMPONENT(YARP_LOGGER_RERUN, "yarp.device.yarpLoggerRerun")

YarpLoggerRerun::YarpLoggerRerun() 
{
}
YarpLoggerRerun::~YarpLoggerRerun() 
{
    close();
}

bool YarpLoggerRerun::open(yarp::os::Searchable& config) 
{
    yCInfo(YARP_LOGGER_RERUN) << "Configuring Rerun visualizer...";

    yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    auto remote = rf.check("remote", yarp::os::Value("/ergocubSim/left_arm")).asString();

    yarp::dev::IPositionControl* iPos{nullptr};
    yarp::dev::IAxisInfo* iAxis{nullptr};

    yarp::os::Property conf;
    conf.put("device", "remote_controlboard");
    conf.put("remote", remote);
    conf.put("local", "/log");
    if (!driver.open(conf)) {
        yCError(YARP_LOGGER_RERUN) << "Failed to connect to" << remote;
        return false;
    }
    if (!(driver.view(iEnc) && driver.view(iPos) && driver.view(iAxis))) {
        yCError(YARP_LOGGER_RERUN) << "Failed to open interfaces";
        driver.close();
        return false;
    }

    double min, max, range;
    iPos->getAxes(&axes);
    axesNames.resize(axes);
    encState.resize(axes);

    yCDebug(YARP_LOGGER_RERUN) << "Position control axes:" << axes;

    for(auto i = 0; i < axes; i++)
    {
        if(!iAxis->getAxisName(i, axesNames[i]))
        {
            yCError(YARP_LOGGER_RERUN) << "Failed to get axis name for axis" << i;
            driver.close();
            return false;
        }
    }
    loadURDF(rr);
    return true;
}

bool YarpLoggerRerun::close() 
{
    return true;
}

// bool YarpLoggerRerun::configure(yarp::os::ResourceFinder& rf)
// {
//     yCInfo(YARP_LOGGER_RERUN) << "Configuring Rerun visualizer...";

//     // yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
//     auto remote = rf.check("remote", yarp::os::Value("/ergocubSim/left_arm")).asString();

//     yarp::dev::IPositionControl* iPos{nullptr};
//     yarp::dev::IAxisInfo* iAxis{nullptr};

//     yarp::os::Property conf;
//     conf.put("device", "remote_controlboard");
//     conf.put("remote", remote);
//     conf.put("local", "/log");
//     if (!driver.open(conf)) {
//         yCError(RERUN_EXAMPLE) << "Failed to connect to" << remote;
//         return false;
//     }
//     if (!(driver.view(iEnc) && driver.view(iPos) && driver.view(iAxis))) {
//         yCError(RERUN_EXAMPLE) << "Failed to open interfaces";
//         driver.close();
//         return false;
//     }

//     double min, max, range;
//     iPos->getAxes(&axes);
//     axesNames.resize(axes);
//     encState.resize(axes);

//     yCDebug(RERUN_EXAMPLE) << "Position control axes:" << axes;

//     for(auto i = 0; i < axes; i++)
//     {
//         if(!iAxis->getAxisName(i, axesNames[i]))
//         {
//             yCError(RERUN_EXAMPLE) << "Failed to get axis name for axis" << i;
//             driver.close();
//             return false;
//         }
//     }
//     loadURDF(rr);
//     return true;
// }

// double YarpLoggerRerun::getPeriod()
// {
//     return 0.1;
// }

// bool YarpLoggerRerun::updateModule() 
// {
//     auto current_time = yarp::os::Time::now();
//     // std::string_view time_key = "yarp_time";
//     // rr.set_time(time_key, current_time);
//     if (encState.size() < axes) {
//         yError() << "encState size mismatch!";
//         return false;
//     }
//     if (!iEnc->getEncoders(encState.data())) {
//         yError() << "Failed to get encoders!";
//         return false;
//     }

//     for (size_t i = 0; i < axesNames.size(); ++i) 
//     {
//         rr.log("ergocubSim/" + axesNames[i], rerun::Scalars(encState[i]));
//     }
//     return true;
// }

void YarpLoggerRerun::loadURDF(rerun::RecordingStream& rr) 
{
    std::string path = "/home/mgloria/iit/ergocub-software/urdf/ergoCub/robots/ergoCubSN002/model.urdf";
    std::string_view name = "ergocubSim";
    rr.log_file_from_path(path, name);
    rr.spawn().exit_on_failure();
}