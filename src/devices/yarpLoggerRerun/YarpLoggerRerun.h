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
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IAxisInfo.h>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

YARP_DECLARE_LOG_COMPONENT(YARP_LOGGER_RERUN)

class YarpLoggerRerun : public yarp::dev::DeviceDriver,
                        public yarp::os::PeriodicThread,
                        public yarp::dev::IMultipleWrapper {
    public:
    YarpLoggerRerun();
    ~YarpLoggerRerun() override;

    bool close() override;
    bool open(yarp::os::Searchable& config) override;
    void run() override;

    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;

    private:
    void configureRerun(rerun::RecordingStream& rr);
    bool attachAllControlBoards(const yarp::dev::PolyDriverList& pList);
    bool loadConfig(yarp::os::Searchable& config);

    rerun::RecordingStream recordingStream{"logger_app_id_" + std::to_string(yarp::os::Time::now()), "logger_recording_id"};
    std::vector<std::string> axesNames;
    yarp::dev::PolyDriver driver;
    yarp::dev::IEncoders* iEnc{nullptr};
    yarp::dev::IMultipleWrapper* iMultWrap{nullptr};
    yarp::dev::IPositionControl* iPos{nullptr};
    yarp::dev::IAxisInfo* iAxis{nullptr};
    std::vector<double> jointsPos, jointsVel, jointsAcc;
    bool logIEncodersOption {false};
    bool logURDFOption {false};
    bool saveToFile{false};
    int axes;
    std::mutex rerunMutex;
    std::string urdfPath, fileName, robotName, urdfFileName{"model.urdf"}, viewerIp, filePath;
};

#endif // YARP_LOGGER_RERUN_H