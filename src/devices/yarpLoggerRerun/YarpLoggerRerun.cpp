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
    if (m_logIMotorEncoders)
    {
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

        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            recordingStream.log("motorEncoders/" + m_axesNames[i], rerun::Scalars(motorPos[i]));
            recordingStream.log("motorVelocities/" + m_axesNames[i], rerun::Scalars(motorVel[i]));
            recordingStream.log("motorAccelerations/" + m_axesNames[i], rerun::Scalars(motorAcc[i]));
        }
    }
    if (m_logIEncoders)
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
        // if (!iEnc->getEncoderAccelerations(jointsAcc.data()))
        // {
        //     yCError(YARP_LOGGER_RERUN) << "Failed to get joints accelerations!";
        //     return;
        // }
        
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            recordingStream.log("encoders/" + m_axesNames[i], rerun::Scalars(jointsPos[i]));
            recordingStream.log("velocities/" + m_axesNames[i], rerun::Scalars(jointsVel[i]));
            // recordingStream.log("accelerations/" + m_axesNames[i], rerun::Scalars(jointsAcc[i]));
        }
    }
    if (m_logIPidControl)
    {
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
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            recordingStream.log("encoders/ref/" + m_axesNames[i], rerun::Scalars(jointPosRef[i]));
            recordingStream.log("positionError/" + m_axesNames[i], rerun::Scalars(jointPosErr[i]));
        }
    }
    if (kinematicsInitialized)
    {
        updateKinematics();
        return;
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
        recordingStream.log_file_from_path(urdfPath, "/", false);
    }
}

bool YarpLoggerRerun::attachAll(const yarp::dev::PolyDriverList& driverList) {
    std::lock_guard<std::mutex> guard(rerunMutex);
    
    this->attachAllControlBoards(driverList);
    if (!initKinematics(urdfPath))
    {
        yCWarning(YARP_LOGGER_RERUN) << "Kinematics initialization failed; proceeding without link pose animation.";
    }
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

bool YarpLoggerRerun::initKinematics(const std::string& urdfPath)
{
    yCInfo(YARP_LOGGER_RERUN) << "Initializing kinematics from URDF:" << urdfPath;
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

    std::vector<double> initPos;
    initPos.resize(axes);
    if (!iEnc->getEncoders(initPos.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get initial encoder values";
        return false;
    }

    for (size_t i = 0; i < m_axesNames.size(); i++)
    {
        jointNameToIdx[m_axesNames[i]] = i;
    }

    iDynTree::Vector3 g; g(0)=0; g(1)=0; g(2)=-9.81;
    iDynTree::Transform world_T_base = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize q(modelLoader.model().getNrOfDOFs());
    q.zero();

    for (size_t i = 0; i < modelLoader.model().getNrOfDOFs(); ++i)
    {
        std::string jointName = kinDyn.model().getJointName(i);
        auto it = jointNameToIdx.find(jointName);
        if (it != jointNameToIdx.end())
        {
            q.setVal(i, iDynTree::deg2rad(-initPos[it->second]));
            yCInfo(YARP_LOGGER_RERUN) << "Setting initial position for joint" << jointName << "to" << q(i) << "rad";
        }
        else
        {
            yCDebug(YARP_LOGGER_RERUN) << "Joint" << jointName << "not in control board, leaving at 0";
        }
    }

    iDynTree::Twist baseVel = iDynTree::Twist::Zero();
    iDynTree::VectorDynSize dq(modelLoader.model().getNrOfDOFs());
    dq.zero();

    if (!kinDyn.setRobotState(world_T_base, q, baseVel, dq, g))
    {
        yCWarning(YARP_LOGGER_RERUN) << "Failed to set initial kinematic state";
        return false;
    }

    auto& model = kinDyn.model();
    model.computeFullTreeTraversal(traversal);
    yCInfo(YARP_LOGGER_RERUN) << traversal.toString(model);

    double rot_angle;
    iDynTree::Vector3 rot_axis;

    for (size_t l = 0; l < model.getNrOfLinks(); l++)
    {
        std::string linkName = model.getLinkName(l);
        const iDynTree::Link* parentLink = traversal.getParentLinkFromLinkIndex(l);
        if (parentLink != nullptr)
        {
            auto transform = kinDyn.getRelativeTransform(model.getLinkName(parentLink->getIndex()), linkName);
            auto position = transform.getPosition();
            auto rotation = transform.getRotation();
            // double qx, qy, qz, qw;
            // rotation.getQuaternion(qw, qx, qy, qz);
            if (!getRotationAxisAndAngle(rotation, rot_axis, rot_angle))
            {
                yCError(YARP_LOGGER_RERUN) << "Failed to extract rotation axis and angle for link" << linkName;
                return false;
            }
            if (rot_angle >= M_PI/2)
            {
                rot_angle = M_PI - rot_angle;
            }
            yCInfo(YARP_LOGGER_RERUN) << "Link" << linkName << "position:" << position.toString() << "rotation axis:" << rot_axis.toString() << "angle(deg):" << rot_angle;

            rerun::components::Translation3D translation(
                static_cast<float>(position(0)) * 0.001f,
                static_cast<float>(position(1)) * 0.001f,
                static_cast<float>(position(2)) * 0.001f
            );
            // rerun::components::RotationQuat rotation_component = rerun::datatypes::Quaternion::from_wxyz(
            //     static_cast<float>(qw),
            //     static_cast<float>(qx),
            //     static_cast<float>(qy),
            //     static_cast<float>(qz)
            // );
            rerun::components::RotationAxisAngle rotation_component = rerun::datatypes::RotationAxisAngle(rerun::datatypes::Vec3D(static_cast<float>(rot_axis(0)), static_cast<float>(rot_axis(1)), static_cast<float>(rot_axis(2))),
                rerun::datatypes::Angle::radians(static_cast<float>(rot_angle)));

            std::string path = getLinkPath(model, linkName);
            // recordingStream.log(path + "/" + linkName, rerun::Transform3D().update_fields().with_translation(translation).with_quaternion(rotation_component));
            recordingStream.log(path + "/" + linkName, rerun::Transform3D().update_fields().with_translation(translation).with_rotation_axis_angle(rotation_component));
        }
    }

    kinematicsInitialized = true;
    yCInfo(YARP_LOGGER_RERUN) << "Kinematics initialized with" << modelLoader.model().getNrOfDOFs() << "DOFs";

    return true;
}

void YarpLoggerRerun::updateKinematics()
{
    if (!kinematicsInitialized)
    {
        return;
    }

    std::vector<double> currentPos;
    currentPos.resize(axes);
    if (!iEnc->getEncoders(currentPos.data()))
    {
        yCError(YARP_LOGGER_RERUN) << "Failed to get current encoder values";
        return;
    }

    iDynTree::Vector3 g; g(0)=0; g(1)=0; g(2)=-9.81;
    iDynTree::Transform world_T_base = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize q(modelLoader.model().getNrOfDOFs());
    q.zero(); 
    for (size_t i = 0; i < modelLoader.model().getNrOfDOFs(); ++i)
    {
        std::string jointName = kinDyn.model().getJointName(i);
        auto it = jointNameToIdx.find(jointName);
        if (it != jointNameToIdx.end())
        {
            q.setVal(i, iDynTree::deg2rad(currentPos[it->second]));
        }
        else
        {
            continue;
        }
    }

    iDynTree::Twist baseVel = iDynTree::Twist::Zero();
    iDynTree::VectorDynSize dq(modelLoader.model().getNrOfDOFs());
    dq.zero();
    if (!kinDyn.setRobotState(world_T_base, q, baseVel, dq, g))
    {
        yCWarning(YARP_LOGGER_RERUN) << "Failed to set initial kinematic state";
        return;
    }

    auto& model = kinDyn.model();
    model.computeFullTreeTraversal(traversal);
    double rot_angle;
    iDynTree::Vector3 rot_axis;

    for (size_t l = 0; l < model.getNrOfLinks(); l++)
    {
        std::string linkName = model.getLinkName(l);
        const iDynTree::Link* parentLink = traversal.getParentLinkFromLinkIndex(l);
        if (parentLink != nullptr)
        {
            auto transform = kinDyn.getRelativeTransform(model.getLinkName(parentLink->getIndex()), linkName);
            auto position = transform.getPosition();
            auto rotation = transform.getRotation();

            // double qx, qy, qz, qw;
            // rotation.getQuaternion(qw, qx, qy, qz);

            if (!getRotationAxisAndAngle(rotation, rot_axis, rot_angle))
            {
                yCError(YARP_LOGGER_RERUN) << "Failed to extract rotation axis and angle for link" << linkName;
                return;
            }
            if (rot_angle >= M_PI/2)
            {
                rot_angle = M_PI - rot_angle;
            }
            rerun::components::Translation3D translation(
                static_cast<float>(position(0)) * 0.001f,
                static_cast<float>(position(1)) * 0.001f,
                static_cast<float>(position(2)) * 0.001f
            );
            // rerun::components::RotationQuat rotation_component = rerun::datatypes::Quaternion::from_wxyz(
            //     static_cast<float>(qw),
            //     static_cast<float>(qx),
            //     static_cast<float>(qy),
            //     static_cast<float>(qz)
            // );
            rerun::components::RotationAxisAngle rotation_component = rerun::datatypes::RotationAxisAngle(rerun::datatypes::Vec3D(static_cast<float>(rot_axis(0)), static_cast<float>(rot_axis(1)), static_cast<float>(rot_axis(2))),
                rerun::datatypes::Angle::radians(static_cast<float>(rot_angle)));

            std::string path = getLinkPath(model, linkName);
            // recordingStream.log(path + "/" + linkName, rerun::Transform3D().update_fields().with_translation(translation).with_quaternion(rotation_component));
            recordingStream.log(path + "/" + linkName, rerun::Transform3D().update_fields().with_translation(translation).with_rotation_axis_angle(rotation_component));            
        }
    }

    return;
}

std::string YarpLoggerRerun::getLinkPath(const iDynTree::Model & model, const std::string & targetLinkName)
{
    std::string path{};
    model.computeFullTreeTraversal(traversal);
    iDynTree::LinkIndex linkIndex = model.getLinkIndex(targetLinkName);
    const iDynTree::Link* parentLink = traversal.getParentLinkFromLinkIndex(linkIndex);

    while (parentLink != nullptr)
    {
        std::string linkName = model.getLinkName(parentLink->getIndex());
        const iDynTree::IJoint* parentJoint = traversal.getParentJointFromLinkIndex(linkIndex);
        if (parentJoint != nullptr)
        {
            std::string jointName = model.getJointName(parentJoint->getIndex());
            path = "/" + jointName + path;
        }
        path = "/" + linkName + path;
        linkIndex = parentLink->getIndex();
        parentLink = traversal.getParentLinkFromLinkIndex(parentLink->getIndex());
    }

    return "/ergoCub" + path;
}

bool YarpLoggerRerun::getRotationAxisAndAngle(const iDynTree::Rotation& rot, iDynTree::Vector3& axis, double& angle)
{
    double s, r1, r2, r3;
    bool ok = rot.getQuaternion(s, r1, r2, r3);

    if (!ok) 
    {
        return false;
    }

    iDynTree::Vector3 imaginary_part;
    imaginary_part(0) = r1;
    imaginary_part(1) = r2;
    imaginary_part(2) = r3;

    double imag_norm_sq = std::pow(r1, 2) + std::pow(r2, 2) + std::pow(r3, 2);
    double imag_norm = std::sqrt(imag_norm_sq);

    double s_clamped = std::min(1.0, std::max(-1.0, s));
    angle = 2.0 * std::acos(s_clamped);

    double epsilon = std::numeric_limits<double>::epsilon();
    if (imag_norm < epsilon) 
    {
        axis(0) = 1.0;
        axis(1) = 0.0;
        axis(2) = 0.0;
        angle = 0.0;
        return true; 
    } 
    else 
    {
        double inverse_norm = 1.0 / imag_norm;
        axis(0) = imaginary_part(0) * inverse_norm;
        axis(1) = imaginary_part(1) * inverse_norm;
        axis(2) = imaginary_part(2) * inverse_norm;
    }

    return true;
}