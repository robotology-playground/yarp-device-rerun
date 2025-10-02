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
#include <cmath>
#include <unordered_map>

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
    std::lock_guard<std::mutex> guard(this->deviceMutex);
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

    // URDF path from configuration or fallback
    urdfPath = config.check("urdf", yarp::os::Value("")).asString();
    if (urdfPath.empty())
    {
        urdfPath = "/home/mgloria/iit/ergocub-software/urdf/ergoCub/robots/ergoCubSN002/model.urdf";
    }

    loadURDF(rr);

    if (!initKinematics(urdfPath))
    {
        yCWarning(YARP_LOGGER_RERUN) << "Kinematics initialization failed; proceeding without link pose animation.";
    }

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
    auto current_time = yarp::os::Time::now();
    // std::string_view time_key = "yarp_time";
    // rr.set_time(time_key, current_time);
    if (jointsPos.size() < axes || jointsVel.size() < axes)
    {
        yCError(YARP_LOGGER_RERUN) << "jointsPos or jointsVel size mismatch!";
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

    for (size_t i = 0; i < axesNames.size(); ++i) 
    {
        rr.log("encoders/" + axesNames[i], rerun::Scalars(jointsPos[i]));
    }

    for (size_t i = 0; i < axesNames.size(); ++i) 
    {
        rr.log("velocities/" + axesNames[i], rerun::Scalars(jointsVel[i]));
    }

    for (size_t i = 0; i < axesNames.size(); ++i) 
    {
        rr.log("accelerations/" + axesNames[i], rerun::Scalars(jointsAcc[i]));
    }
    // animateURDF();
}

void YarpLoggerRerun::loadURDF(rerun::RecordingStream& rr) 
{
    std::string path = "/home/mgloria/iit/ergocub-software/urdf/ergoCub/robots/ergoCubSN002/model.urdf";
    std::string_view name = "ergocubSim/";
    rr.log_file_from_path(path, name, false);
    rr.spawn().exit_on_failure();
}

void YarpLoggerRerun::animateURDF()
{
    // Set a time sequence so Rerun can build a timeline.
    // rr.set_time_sequence("frame", static_cast<int64_t>(frameCounter++));

    if (joints.empty())
    {
        joints.reserve(axesNames.size());
        for (const auto &name : axesNames)
        {

            joints.push_back(JointInfo{ name, {0.f, 0.f, 1.f} }); // FIXME (deve prenderselo dall'urdf)
        }
    }

    if (kinematicsInitialized)
    {
        updateLinkPosesFromEncoders();
        return;
    }

    // const size_t n = std::min(jointsPos.size(), joints.size());
    // for (size_t i = 0; i < n; ++i)
    // {
    //     float angle_rad = static_cast<float>(jointsPos[i] * M_PI / 180.0);
    //     auto &j = joints[i];
    //     rr.log(
    //         std::string("ergocubSim/encoders/") + j.name,
    //         rerun::Transform3D::from_rotation(
    //             rerun::RotationAxisAngle(j.axis, rerun::Angle::radians(angle_rad))
    //         ).with_relation(rerun::components::TransformRelation::ChildFromParent)
    //     );
    // }
}

bool YarpLoggerRerun::initKinematics(const std::string& urdfPath)
{
    if (urdfPath.empty())
    {
        yCError(YARP_LOGGER_RERUN) << "Empty URDF path";
        return false;
    }
    if (!modelLoader.loadModelFromFile(urdfPath))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to load URDF:" << urdfPath;
        return false;
    }
    if (!kinDyn.loadRobotModel(modelLoader.model()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to load model into KinDynComputations";
        return false;
    }

    // Setup initial state
    iDynTree::Vector3 g; g(0)=0; g(1)=0; g(2)=-9.81;
    iDynTree::Transform world_T_base = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize q(modelLoader.model().getNrOfDOFs());

    for (size_t i = 0; i < modelLoader.model().getNrOfDOFs(); ++i)
    {
        q(i) = 0.0;
    }

    iDynTree::Twist baseVel = iDynTree::Twist::Zero();
    iDynTree::VectorDynSize dq(modelLoader.model().getNrOfDOFs()); 
    for (size_t i = 0; i < modelLoader.model().getNrOfDOFs(); ++i)
    {
        dq(i) = 0.0;
    }

    if (!kinDyn.setRobotState(world_T_base, q, baseVel, dq, g))
    {
        yCWarning(YARP_LOGGER_RERUN) << "Failed to set initial kinematic state";
    }

    kinematicsInitialized = true;
    yCInfo(YARP_LOGGER_RERUN) << "Kinematics initialized with" << modelLoader.model().getNrOfDOFs() << "DOFs";

    return true;
}

void YarpLoggerRerun::updateLinkPosesFromEncoders()
{
    size_t dofs = modelLoader.model().getNrOfDOFs();
    if (dofs == 0)
    {
        return;
    }

    iDynTree::VectorDynSize q(dofs);
    static std::unordered_map<std::string,size_t> jointNameToIdx;
    if (jointNameToIdx.empty())
    {
        for (size_t i = 0; i < dofs; i++)
        {
            jointNameToIdx[kinDyn.model().getJointName(i)] = i;
        }
    }
    for (size_t i = 0; i < dofs; i++) 
    {
        q(i) = 0.0;
    }
    for (size_t i = 0; i < axesNames.size(); ++i)
    {
        auto it = jointNameToIdx.find(axesNames[i]);
        if (it != jointNameToIdx.end())
        {
            q(it->second) = jointsPos[i] * M_PI / 180.0; // deg->rad
        }
    }
    iDynTree::Twist baseVel = iDynTree::Twist::Zero();
    iDynTree::Vector3 g; g(0)=0; g(1)=0; g(2)=-9.81;
    iDynTree::VectorDynSize dq(dofs); 
    for (size_t i = 0; i < dofs; i++) 
    {
        dq(i) = 0.0;
    }
    iDynTree::Transform world_T_base = iDynTree::Transform::Identity();
    kinDyn.setRobotState(world_T_base, q, baseVel, dq, g);

    auto& model = kinDyn.model();
    size_t nLinks = model.getNrOfLinks();
    for (size_t l = 0; l < nLinks; l++)
    {
        auto linkName = model.getLinkName(l);
        auto T = kinDyn.getWorldTransform(linkName);
        auto p = T.getPosition();
        auto R = T.getRotation();
        double qx, qy, qz, qw; 

        R.getQuaternion(qx, qy, qz, qw);
        // rr.log(
        //     std::string("ergocubSim/encoders/") + linkName,
        //     rerun::Transform3D()
        //         .with_translation({static_cast<float>(p(0)), static_cast<float>(p(1)), static_cast<float>(p(2))})
        //         .with_rotation(rerun::Quaternion{static_cast<float>(qw), static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz)})
        //         // .with_relation(rerun::components::TransformRelation::ChildFromParent)
        // );

    }
    rr.log(
        "ergocubSim/head",
        rerun::Transform3D()
            .with_translation({0.f, 0.f, 0.f})
            .with_rotation(rerun::Quaternion{1.f, 0.f, 0.f, 0.f})
            .with_relation(rerun::components::TransformRelation::ChildFromParent)
    );
}