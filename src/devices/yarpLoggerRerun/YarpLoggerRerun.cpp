/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Vocab.h>
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
    if (conf.check("logILocalization2D") && m_logILocalization2D)
    {
        yarp::os::Property loc2DClientProp;
        loc2DClientProp.put("device", "localization2D_nwc_yarp");
        loc2DClientProp.put("remote", m_localizationRemoteName);
        loc2DClientProp.put("local",  "/yarpLoggerRerun" + m_localizationRemoteName + "/client");
        if (!localization2DClient.open(loc2DClientProp))
        {
            yCError(YARP_LOGGER_RERUN) << "Failed to open the localization2DClient driver.";
            return false;
        }
        if (!localization2DClient.view(iLoc))
        {
            yCError(YARP_LOGGER_RERUN) << "Failed to open ILocalization2D interface.";
            return false;
        }
    }
    if (conf.check("logIRawValuesPublisher") && m_logIRawValuesPublisher)
    {
        yarp::os::Property rawValPubClientProp;
        rawValPubClientProp.put("device", "rawValuesPublisherClient");
        rawValPubClientProp.put("remote", m_rawValuesPublisherRemoteName);
        rawValPubClientProp.put("local",  "/yarpLoggerRerun" + m_rawValuesPublisherRemoteName + "/client");
        if (!rawValuesPublisherClient.open(rawValPubClientProp))
        {
            yCError(YARP_LOGGER_RERUN) << "Failed to open the rawValuesPublisherClient driver.";
            return false;
        }
        if (!rawValuesPublisherClient.view(iRawValPub))
        {
            yCError(YARP_LOGGER_RERUN) << "Failed to open IRawValuesPublisher interface.";
            return false;
        }
    }

    if (!(driver.view(iEnc) && driver.view(iPos) && driver.view(iMotorEnc) && driver.view(iPid) && driver.view(iAxis) && driver.view(iMultWrap) && driver.view(iTorque) && driver.view(iAmp) && driver.view(iCtrlMode) && driver.view(iIntMode) && driver.view(iMotor)))
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
    jointsInteractionModes.resize(axes);
    motorTemperatures.resize(axes);
    odometryData.resize(9); // x, y, theta, base_vel_x, base_vel_y, base_vel_theta, odom_vel_x, odom_vel_y, odom_vel_theta
    rawDataValuesMap.clear();

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
    if (m_logILocalization2D && localization2DClient.isValid())
    {
        localization2DClient.close();
    }
    if (m_logIRawValuesPublisher && rawValuesPublisherClient.isValid())
    {
        rawValuesPublisherClient.close();
    }
    return true;
}

void YarpLoggerRerun::run()
{
    if (m_logIMotorEncoders)
    {
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            if (!iMotorEnc->getMotorEncoder(i, &motorPos[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get motor positions for joint" << m_axesNames[i] << ", setting to 0.0";
                motorPos[i] = 0.0;
            }
            if (!iMotorEnc->getMotorEncoderSpeed(i, &motorVel[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get motor velocities for joint" << m_axesNames[i] << ", setting to 0.0";
                motorVel[i] = 0.0;
            }
            if (!iMotorEnc->getMotorEncoderAcceleration(i, &motorAcc[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get motor accelerations for joint" << m_axesNames[i] << ", setting to 0.0";
                motorAcc[i] = 0.0;
            }
            recordingStream.try_log("motorEncoders/" + m_axesNames[i], rerun::Scalars(motorPos[i]));
            recordingStream.try_log("motorVelocities/" + m_axesNames[i], rerun::Scalars(motorVel[i]));
            recordingStream.try_log("motorAccelerations/" + m_axesNames[i], rerun::Scalars(motorAcc[i]));
        }
    }
    if (m_logIEncoders)
    {
        for (size_t i = 0; i < axes; ++i)
        {
            if (!iEnc->getEncoder(i, &jointsPos[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get joints positions for joint" << m_axesNames[i] << ", setting to 0.0";
                jointsPos[i] = 0.0;
            }
            if (!iEnc->getEncoderSpeed(i, &jointsVel[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get joints velocities for joint" << m_axesNames[i] << ", setting to 0.0";
                jointsVel[i] = 0.0;
            }
            if(!iEnc->getEncoderAcceleration(i, &jointsAcc[i]))
            {
                // Not implemented for the fingers coupling, see:
                // https://github.com/icub-tech-iit/ergocub-software/blob/13218d0168d43af01e0b794c87198bcb48bf4105/src/modules/couplingXCubHandMk5/CouplingXCubHandMk5.cpp#L196-L198
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get joints accelerations for joint" << m_axesNames[i] << ", setting to 0.0";
                jointsAcc[i] = 0.0;
            }
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
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get joints position references for joint" << m_axesNames[i] << ", setting to 0.0";
                jointPosRef[i] = 0.0;
            }
            if (!iPid->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, i, &jointPosErr[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get joints position errors for joint" << m_axesNames[i] << ", setting to 0.0";
                jointPosErr[i] = 0.0;
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
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get joints torques for joint" << m_axesNames[i] << ", setting to 0.0";
                jointsTorques[i] = 0.0;
            }
            recordingStream.try_log("torques/" + m_axesNames[i], rerun::Scalars(jointsTorques[i]));
        }
    }
    if (m_logIAmplifierControl)
    {
        // Those methods are not implemented in gz-sim-yarp-plugins, logIAmplifierControl should be used only with real robots

        for (size_t i = 0; i < motorPWM.size(); ++i) 
        {
            if (!iAmp->getCurrent(i, &motorCurrents[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get motor currents for motor" << m_axesNames[i] << ", setting to 0.0";
                motorCurrents[i] = 0.0;
            }
            if (!iAmp->getPWM(i, &motorPWM[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get motor PWMs for motor" << m_axesNames[i] << ", setting to 0.0";
                motorPWM[i] = 0.0;
            }
            recordingStream.try_log("motorCurrents/" + m_axesNames[i], rerun::Scalars(motorCurrents[i]));
            recordingStream.try_log("motorPWM/" + m_axesNames[i], rerun::Scalars(motorPWM[i]));
        }
    }
    if (m_logIControlMode)
    {
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            int mode;
            if (!iCtrlMode->getControlMode(i, &mode))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get control mode for joint" << m_axesNames[i] << ", setting to UNKNOWN";
                jointsCtrlModes[i] = VOCAB_CM_UNKNOWN;
            }
            jointsCtrlModes[i] = yarp::os::Vocab32::decode(mode);
            recordingStream.try_log_static("controlModes/" + m_axesNames[i], rerun::TextLog(rerun::components::Text(jointsCtrlModes[i])).with_level(rerun::TextLogLevel::Info));
        }
    }
    if (m_logIInteractionMode)
    {
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            yarp::dev::InteractionModeEnum mode;
            if (!iIntMode->getInteractionMode(i, &mode))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get interaction mode for joint" << m_axesNames[i] << ", setting to UNKNOWN";
                jointsInteractionModes[i] = yarp::dev::VOCAB_IM_UNKNOWN;
            }
            jointsInteractionModes[i] = yarp::os::Vocab32::decode(mode);
            recordingStream.try_log_static("interactionModes/" + m_axesNames[i], rerun::TextLog(rerun::components::Text(jointsInteractionModes[i])).with_level(rerun::TextLogLevel::Info));
        }
    }
    if (m_logIMotorTemperatures)
    {
        // Available only with this yarp branch: https://github.com/ami-iit/yarp/tree/yarp-3.10.1-motor-temperature
        // See this PR for more details: https://github.com/robotology/yarp/pull/3188
        for (size_t i = 0; i < m_axesNames.size(); ++i) 
        {
            if (!iMotor->getTemperature(i, &motorTemperatures[i]))
            {
                yCWarningOnce(YARP_LOGGER_RERUN) << "Failed to get motor temperatures for motor" << m_axesNames[i] << ", setting to 0.0";
                motorTemperatures[i] = 0.0;
            }
            recordingStream.try_log("motorTemperatures/" + m_axesNames[i], rerun::Scalars(motorTemperatures[i]));
        }
    }
    if (m_logILocalization2D)
    {
        yarp::dev::OdometryData odomData;
        if (!iLoc->getEstimatedOdometry(odomData))
        {
            yCWarning(YARP_LOGGER_RERUN) << "Odometry data was not read correctly";
        }

        recordingStream.try_log("odom/pose", rerun::Transform3D(rerun::components::Translation3D(static_cast<float>(odomData.odom_x), static_cast<float>(odomData.odom_y), 0.0f), rerun::components::RotationAxisAngle(rerun::datatypes::RotationAxisAngle(rerun::datatypes::Vec3D(0.0f, 0.0f, 1.0f), rerun::datatypes::Angle::degrees(static_cast<float>(odomData.odom_theta))))));
        recordingStream.try_log("odom/base_velocity", rerun::Arrows3D::from_vectors(rerun::datatypes::Vec3D(static_cast<float>(odomData.base_vel_x), static_cast<float>(odomData.base_vel_y), 0.0f)).with_origins(rerun::datatypes::Vec3D(static_cast<float>(odomData.odom_x), static_cast<float>(odomData.odom_y), 0.0f)));
        recordingStream.try_log("odom/odom_velocity", rerun::Arrows3D::from_vectors(rerun::datatypes::Vec3D(static_cast<float>(odomData.odom_vel_x), static_cast<float>(odomData.odom_vel_y), 0.0f)).with_origins(rerun::datatypes::Vec3D(static_cast<float>(odomData.odom_x), static_cast<float>(odomData.odom_y), 0.0f)));
    }
    if (m_logIRawValuesPublisher)
    {
        if (!iRawValPub->getRawDataMap(rawDataValuesMap))
        {
            yCWarning(YARP_LOGGER_RERUN) << "Raw data was not read correctly";
        }

        for (auto [key,value] : rawDataValuesMap)
        {
            recordingStream.try_log("rawValues/" + key, rerun::Scalars(value));
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
            recordingStream.try_log(path + "/" + linkName, rerun::Transform3D().with_translation(translation).with_quaternion(rotation_component));
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
            recordingStream.try_log(path + "/" + linkName, rerun::Transform3D().with_translation(translation).with_quaternion(rotation_component));        
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