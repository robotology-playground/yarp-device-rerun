/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/ResourceFinder.h>
#include <rerun/log_sink.hpp>

#include "YarpLoggerRerun.h"

YARP_LOG_COMPONENT(YARP_LOGGER_RERUN, "yarp.device.yarpLoggerRerun")
constexpr double log_thread_default{ 0.010 };

YarpLoggerRerun::YarpLoggerRerun() : yarp::os::PeriodicThread(log_thread_default)
{

}
YarpLoggerRerun::~YarpLoggerRerun() 
{

}

bool YarpLoggerRerun::open(yarp::os::Searchable& config) 
{
    yCInfo(YARP_LOGGER_RERUN) << "Configuring Rerun visualizer...";

    if(!parseParams(config)) {
        yCError(YARP_LOGGER_RERUN) << "Error parsing parameters";
        return false;
    }

    yarp::os::Property conf;
    conf.fromString(config.toString().c_str());
    conf.put("device", "controlboardremapper");

    if (!driver.open(conf))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to open controlBoard driver";
        return false;
    }

    if (!(driver.view(iEnc) && driver.view(iPos) && driver.view(iMotorEnc) && driver.view(iPid) && driver.view(iAxis) && driver.view(iMultWrap)))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to open interfaces";
        driver.close();
        return false;
    }

    iPos->getAxes(&axes);
    jointsPos.resize(axes);
    jointsVel.resize(axes);
    jointsAcc.resize(axes);
    motorPos.resize(axes);
    motorVel.resize(axes);
    motorAcc.resize(axes);
    jointPosRef.resize(axes);
    jointPosErr.resize(axes);

    yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    urdfPath = rf.findFileByName(urdfFileName);

    configureRerun(recordingStream);
    
    return true;
}

bool YarpLoggerRerun::close() 
{
    if (isRunning())
    {
        stop();
    }

    if (driver.isValid())
    {
        driver.close();
    }
    return true;
}

void YarpLoggerRerun::run()
{
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

    if (!iMotorEnc->getMotorEncoders(motorPos.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get motor encoders!";
        return;
    }
    if (!iMotorEnc->getMotorEncoderSpeeds(motorVel.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get motor velocities!";
        return;
    }
    if (!iMotorEnc->getMotorEncoderAccelerations(motorAcc.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get motor accelerations!";
        return;
    }
    
    if (!iPid->getPidReferences(yarp::dev::VOCAB_PIDTYPE_POSITION, jointPosRef.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get joints position references!";
        return;
    }
    if (!iPid->getPidErrors(yarp::dev::VOCAB_PIDTYPE_POSITION, jointPosErr.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get joints position errors!";
        return;
    }

    if (m_logIMotorEncoders)
    {
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            recordingStream.log("motorEncoders/" + m_axesNames[i], rerun::Scalars(motorPos[i]));
            recordingStream.log("motorVelocities/" + m_axesNames[i], rerun::Scalars(motorVel[i]));
            recordingStream.log("motorAccelerations/" + m_axesNames[i], rerun::Scalars(motorAcc[i]));
        }
    }
    if (m_logIEncoders)
    {
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            recordingStream.log("encoders/" + m_axesNames[i], rerun::Scalars(jointsPos[i]));
            recordingStream.log("velocities/" + m_axesNames[i], rerun::Scalars(jointsVel[i]));
            recordingStream.log("accelerations/" + m_axesNames[i], rerun::Scalars(jointsAcc[i]));
        }
    }
    if (m_logIPidControl)
    {
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            recordingStream.log("encoders/ref/" + m_axesNames[i], rerun::Scalars(jointPosRef[i]));
            recordingStream.log("positionError/" + m_axesNames[i], rerun::Scalars(jointPosErr[i]));
        }
    }
}

void YarpLoggerRerun::configureRerun(rerun::RecordingStream& recordingStream) 
{
    recordingStream.spawn();
    if (m_saveToFile && !m_fileName.empty())
    {
        yCInfo(YARP_LOGGER_RERUN) << "Start saving log to file";
        recordingStream.set_sinks(rerun::GrpcSink{"rerun+http://" + m_viewerIp + ":9876/proxy"}, rerun::FileSink{m_filePath + m_fileName + ".rrd"});
    }
    else
    {
        yCInfo(YARP_LOGGER_RERUN) << "Only realtime streaming, no file saving";
        recordingStream.set_sinks(rerun::GrpcSink{"rerun+http://" + m_viewerIp + ":9876/proxy"});
    }

    if (m_logURDF)
    {
        recordingStream.log_file_from_path(urdfPath, robotName, true);
    }
}

bool YarpLoggerRerun::attachAll(const yarp::dev::PolyDriverList& driverList) {
    std::lock_guard<std::mutex> guard(rerunMutex);
    
    this->attachAllControlBoards(driverList);
    this->start();

    return true;
}

bool YarpLoggerRerun::attachAllControlBoards(const yarp::dev::PolyDriverList& pList) 
{
    yCInfo(YARP_LOGGER_RERUN) << "Attaching all control boards";
    yarp::dev::PolyDriverList controlBoardList;
    for (size_t devIdx = 0; devIdx < (size_t)pList.size(); devIdx++)
    {
        yarp::dev::IEncoders* pEncs = 0;
        if (pList[devIdx]->poly->view(pEncs))
        {
            controlBoardList.push(const_cast<yarp::dev::PolyDriverDescriptor&>(*pList[devIdx]));
        }
    }

    if (!iMultWrap->attachAll(controlBoardList))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to attach all control boards";
        return false;
    }
    return true;
}

bool YarpLoggerRerun::detachAll() 
{
    std::lock_guard<std::mutex> rerunGuard(rerunMutex);
    if (isRunning())
    {
        stop();
    }
    return iMultWrap->detachAll();
}
