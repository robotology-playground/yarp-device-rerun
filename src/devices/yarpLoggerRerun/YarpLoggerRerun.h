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
#include <yarp/os/LogStream.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

#include "YarpLoggerRerun_ParamsParser.h"

YARP_DECLARE_LOG_COMPONENT(YARP_LOGGER_RERUN)

class YarpLoggerRerun : public yarp::dev::DeviceDriver, public yarp::os::PeriodicThread, public YarpLoggerRerun_ParamsParser {
    public:
    YarpLoggerRerun();
    ~YarpLoggerRerun() override;

    bool close() override;
    bool open(yarp::os::Searchable& config) override;
    void run() override;

    private:
    void configureRerun(rerun::RecordingStream& rr);

    rerun::RecordingStream recordingStream{"logger_app_id_" + std::to_string(yarp::os::Time::now()), "logger_recording_id"};
    std::vector<std::string> axesNames;
    yarp::dev::PolyDriver driver;
    yarp::dev::IEncoders* iEnc{nullptr};
    std::vector<double> jointsPos, jointsVel, jointsAcc;
    int axes;
    std::mutex rerunMutex;
    std::string urdfPath;
};

#endif // YARP_LOGGER_RERUN_H