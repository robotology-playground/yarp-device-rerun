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

    if (!(driver.view(iEnc) && driver.view(iPos) && driver.view(iMotorEnc) && driver.view(iPid) && driver.view(iAxis) && driver.view(iMultWrap) && driver.view(iTorque) && driver.view(iAmp) && driver.view(iCtrlMode)))
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
    jointsTorques.resize(axes);
    motorCurrents.resize(axes);
    motorPWM.resize(axes);
    jointsCtrlModes.resize(axes);

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
            recordingStream.try_log("motorEncoders/" + m_axesNames[i], rerun::Scalars(motorPos[i]));
            recordingStream.try_log("motorVelocities/" + m_axesNames[i], rerun::Scalars(motorVel[i]));
            recordingStream.try_log("motorAccelerations/" + m_axesNames[i], rerun::Scalars(motorAcc[i]));
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
        for (size_t i = 0; i < axes; ++i)
        {
            if(!iEnc->getEncoderAcceleration(i, &jointsAcc[i]))
            {
                // Not implemented for the fingers coupling, see:
                // https://github.com/icub-tech-iit/ergocub-software/blob/13218d0168d43af01e0b794c87198bcb48bf4105/src/modules/couplingXCubHandMk5/CouplingXCubHandMk5.cpp#L196-L198
                yCErrorOnce(YARP_LOGGER_RERUN) << "Failed to get joints accelerations for joint" << m_axesNames[i];
                jointsAcc[i] = 0.0;
            }
        }
        
        for (size_t i = 0; i < axes; ++i) 
        {
            recordingStream.try_log("encoders/" + m_axesNames[i], rerun::Scalars(jointsPos[i]));
            recordingStream.try_log("velocities/" + m_axesNames[i], rerun::Scalars(jointsVel[i]));
            recordingStream.try_log("accelerations/" + m_axesNames[i], rerun::Scalars(jointsAcc[i]));
        }
    }
    if (m_logIPidControl)
    {
        // Those methods are not implemented in gz-sim-yarp-plugins, logIPidControl should be used only with real robots
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            if (!iPid->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, i, &jointPosRef[i]))
            {
                yCErrorOnce(YARP_LOGGER_RERUN) << "Failed to get joints position references for joint" << m_axesNames[i];
            }
            if (!iPid->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, i, &jointPosErr[i]))
            {
                yCErrorOnce(YARP_LOGGER_RERUN) << "Failed to get joints position errors for joint" << m_axesNames[i];
            }
            recordingStream.try_log("encoders/ref/" + m_axesNames[i], rerun::Scalars(jointPosRef[i]));
            recordingStream.try_log("positionError/" + m_axesNames[i], rerun::Scalars(jointPosErr[i]));
        }
    }
    if (m_logITorqueControl)
    {
        for (size_t i = 0; i < m_axesNames.size(); ++i)
        {
            if (!iTorque->getTorque(i, &jointsTorques[i]))
            {
                // Not implemented for the fingers coupling, see:
                // https://github.com/icub-tech-iit/ergocub-software/blob/13218d0168d43af01e0b794c87198bcb48bf4105/src/modules/couplingXCubHandMk5/CouplingXCubHandMk5.cpp#L201-L204
                yCErrorOnce(YARP_LOGGER_RERUN) << "Failed to get joints torques for joint" << m_axesNames[i];
            }
            recordingStream.try_log("torques/" + m_axesNames[i], rerun::Scalars(jointsTorques[i]));
        }
    }
    if (m_logIAmplifierControl)
    {
        // Those methods are not implemented in gz-sim-yarp-plugins, logIAmplifierControl should be used only with real robots
        if (!iAmp->getCurrents(motorCurrents.data()))
        {
            yCError(YARP_LOGGER_RERUN) << "Failed to get motor currents!";
            return;
        }
        for (size_t i = 0; i < motorPWM.size(); ++i) 
        {
            if (!iAmp->getPWM(i, &motorPWM[i]))
            {
                yCError(YARP_LOGGER_RERUN) << "Failed to get motor PWMs!";
                return;
            }
        }
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            recordingStream.try_log("motorCurrents/" + m_axesNames[i], rerun::Scalars(motorCurrents[i]));
            recordingStream.try_log("motorPWM/" + m_axesNames[i], rerun::Scalars(motorPWM[i]));
        }
    }
    if(m_logIControlMode)
    {
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            int mode;
            if (!iCtrlMode->getControlMode(i, &mode))
            {
                yCErrorOnce(YARP_LOGGER_RERUN) << "Failed to get control mode for joint" << m_axesNames[i];
                return;
            }
            jointsCtrlModes[i] = yarp::os::Vocab32::decode(mode);
            recordingStream.try_log_static("controlModes/" + m_axesNames[i], rerun::TextLog(rerun::components::Text(jointsCtrlModes[i])).with_level( rerun::TextLogLevel::Info));
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
        recordingStream.try_log_file_from_path(urdfPath, "/", false);
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
    initPos.resize(m_axesNames.size());
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
            q.setVal(i, iDynTree::deg2rad(initPos[it->second]));
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
    zeroTransforms.resize(model.getNrOfLinks());

    for (size_t l = 0; l < model.getNrOfLinks(); l++)
    {
        std::string linkName = model.getLinkName(l);
        const iDynTree::Link* parentLink = traversal.getParentLinkFromLinkIndex(l);
        if (parentLink != nullptr)
        {
            // Rerun allows to publish on the child node the trasform between the child link frame actual position and the child link frame position if the joint position is set to 0
            // see https://github.com/rerun-io/rerun/issues/10626#issuecomment-3517446719
            auto actualTransform = kinDyn.getRelativeTransform(model.getLinkName(parentLink->getIndex()), linkName); //child link actual pose
            const iDynTree::IJoint* parentJoint = traversal.getParentJointFromLinkIndex(l);
            if (parentJoint != nullptr)
            {
                zeroTransforms[l] = parentJoint->getRestTransform(parentLink->getIndex(), l); //child link pose at zero joint position
            }
            else
            {
                yCError(YARP_LOGGER_RERUN) << "Parent joint is null for link" << linkName;
                return false;
            }
            auto transform = zeroTransforms[l].inverse() * actualTransform;
            auto position = transform.getPosition();
            auto rotation = transform.getRotation();
            double qx, qy, qz, qw;
            rotation.getQuaternion(qw, qx, qy, qz);

            rerun::components::Translation3D translation(
                static_cast<float>(position(0)) * 0.001f,
                static_cast<float>(position(1)) * 0.001f,
                static_cast<float>(position(2)) * 0.001f
            );
            rerun::components::RotationQuat rotation_component = rerun::datatypes::Quaternion::from_wxyz(
                static_cast<float>(qw),
                static_cast<float>(qx),
                static_cast<float>(qy),
                static_cast<float>(qz)
            );

            std::string path = getLinkPath(model, linkName);
            recordingStream.try_log(path + "/" + linkName, rerun::Transform3D().update_fields().with_translation(translation).with_quaternion(rotation_component));
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
    currentPos.resize(m_axesNames.size());
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

    for (size_t l = 0; l < model.getNrOfLinks(); l++)
    {
        std::string linkName = model.getLinkName(l);
        const iDynTree::Link* parentLink = traversal.getParentLinkFromLinkIndex(l);
        if (parentLink != nullptr)
        {
            auto actualTransform = kinDyn.getRelativeTransform(model.getLinkName(parentLink->getIndex()), linkName); 
            auto transform = zeroTransforms[l].inverse() * actualTransform;
            auto position = transform.getPosition();
            auto rotation = transform.getRotation();

            double qx, qy, qz, qw;
            rotation.getQuaternion(qw, qx, qy, qz);

            rerun::components::Translation3D translation(
                static_cast<float>(position(0)) * 0.001f,
                static_cast<float>(position(1)) * 0.001f,
                static_cast<float>(position(2)) * 0.001f
            );
            rerun::components::RotationQuat rotation_component = rerun::datatypes::Quaternion::from_wxyz(
                static_cast<float>(qw),
                static_cast<float>(qx),
                static_cast<float>(qy),
                static_cast<float>(qz)
            );

            std::string path = getLinkPath(model, linkName);
            recordingStream.try_log(path + "/" + linkName, rerun::Transform3D().update_fields().with_translation(translation).with_quaternion(rotation_component));        
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