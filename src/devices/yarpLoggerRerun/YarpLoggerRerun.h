/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_LOGGER_RERUN_H
#define YARP_LOGGER_RERUN_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/LogComponent.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
#include <array>

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Traversal.h>

#include "YarpLoggerRerun_ParamsParser.h"

YARP_DECLARE_LOG_COMPONENT(YARP_LOGGER_RERUN)

class YarpLoggerRerun : public yarp::dev::DeviceDriver, public yarp::os::PeriodicThread, public YarpLoggerRerun_ParamsParser {
    public:
    YarpLoggerRerun();
    ~YarpLoggerRerun() override;

    //DeviceDriver
    bool close() override;
    /**
        * Configure with a set of options.
        * @param config The options to use
        * @return true iff the object could be configured.
        */
    bool open(yarp::os::Searchable& config) override;
    void run() override;

    private:
    void loadURDF(rerun::RecordingStream& rr);
    void animateURDF();
    bool initKinematics(const std::string& urdfPath);
    void updateLinkPosesFromEncoders();
        rerun::RecordingStream rr{"rerun_example_" + std::to_string(yarp::os::Time::now()), "id_example"};
        std::vector<std::string> axesNames;
        yarp::dev::PolyDriver driver;
        yarp::dev::IEncoders* iEnc{nullptr};
        std::vector<double> jointsPos, jointsVel, jointsAcc;
        int axes;
        std::mutex deviceMutex;
        std::mutex rerunMutex;
        struct JointInfo {
            std::string name;               // Joint (and corresponding link) name
            std::array<float,3> axis;        // Rotation axis in parent frame
        };
        std::vector<JointInfo> joints;       // One entry per controllable axis
        size_t frameCounter{0};              // Monotonic frame counter for time sequence
    std::string urdfPath;                // Configured URDF path

    iDynTree::ModelLoader modelLoader;
    iDynTree::KinDynComputations kinDyn;
    bool kinematicsInitialized{false};
};

#endif // YARP_LOGGER_RERUN_H