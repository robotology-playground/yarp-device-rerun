/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IAxisInfo.h>
#include <rerun/log_sink.hpp>

#include "YarpLoggerRerun.h"

YARP_LOG_COMPONENT(YARP_LOGGER_RERUN, "yarp.device.yarpLoggerRerun")
constexpr double log_thread_default{ 0.010 };

YarpLoggerRerun::YarpLoggerRerun() : yarp::os::PeriodicThread(log_thread_default)
{

}
YarpLoggerRerun::~YarpLoggerRerun() 
{
    close();
}

bool YarpLoggerRerun::open(yarp::os::Searchable& config) 
{
    yCInfo(YARP_LOGGER_RERUN) << "Configuring Rerun visualizer...";
    
    if(!parseParams(config)) {
        yCError(YARP_LOGGER_RERUN) << "Error parsing parameters";
        return false;
    }

    std::string remote_port = "/" + m_robot + "/" + m_remote;
    yarp::dev::IPositionControl* iPos{nullptr};
    yarp::dev::IAxisInfo* iAxis{nullptr};

    yarp::os::Property conf;
    conf.put("device", "remote_controlboard");
    conf.put("remote", remote_port);
    conf.put("local", "/log");
    if (!driver.open(conf))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to connect to" << remote_port;
        return false;
    }
    if (!(driver.view(iEnc) && driver.view(iPos) && driver.view(iAxis)))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to open interfaces";
        driver.close();
        return false;
    }

    double min, max, range;
    iPos->getAxes(&axes);
    axesNames.resize(axes);
    jointsPos.resize(axes);
    jointsVel.resize(axes);
    jointsAcc.resize(axes);

    for (auto i = 0; i < axes; i++)
    {
        if (!iAxis->getAxisName(i, axesNames[i]))
        {
            yCError(YARP_LOGGER_RERUN) << "Failed to get axis name for axis" << i;
            driver.close();
            return false;
        }
    }

    yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    urdfPath = rf.findFileByName(m_urdf);

    loadURDF(rr);

    this->start();
    return true;
}

bool YarpLoggerRerun::close() 
{
    if (isRunning())
    {
        stop();
    }
    return true;
}

void YarpLoggerRerun::run()
{
    std::lock_guard<std::mutex> rerunGuard(rerunMutex);
    auto current_time = yarp::os::Time::now();

    if (jointsPos.size() < axes || jointsVel.size() < axes || jointsAcc.size() < axes)
    {
        yCError(YARP_LOGGER_RERUN) << "jointsPos, jointsVel or jointsAcc size mismatch!";
        return;
    }
    if (!iEnc->getEncoders(jointsPos.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get encoders!";
        return;
    }
    if (!iEnc->getEncoderSpeeds(jointsVel.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get joints velocities!";
        return;
    }
    if (!iEnc->getEncoderAccelerations(jointsAcc.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get joints accelerations!";
        return;
    }

    if (m_logIEncoders)
    {
        for (size_t i = 0; i < axesNames.size(); ++i) 
        {
            rr.log("encoders/" + axesNames[i], rerun::Scalars(jointsPos[i]));
            rr.log("velocities/" + axesNames[i], rerun::Scalars(jointsVel[i]));
            rr.log("accelerations/" + axesNames[i], rerun::Scalars(jointsAcc[i]));
        }
    }
}

void YarpLoggerRerun::loadURDF(rerun::RecordingStream& rr) 
{
    std::lock_guard<std::mutex> rerunGuard(rerunMutex);

    if (m_saveToFile)
    {
        yCInfo(YARP_LOGGER_RERUN) << "Start saving log to file";
        std::string file_name = "my_log.rrd";
        rr.set_sinks(rerun::GrpcSink{"rerun+http://" + m_viewer_ip + ":9876/proxy"}, rerun::FileSink{file_name});
    }
    else
    {
        yCInfo(YARP_LOGGER_RERUN) << "Only realtime streaming, no file saving";
        rr.set_sinks(rerun::GrpcSink{"rerun+http://" + m_viewer_ip + ":9876/proxy"});
    }
    rr.log_file_from_path(urdfPath, m_robot, false);
}