/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef RERUN_EXAMPLE_H
#define RERUN_EXAMPLE_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

#include <urdf_parser/urdf_parser.h>


YARP_DECLARE_LOG_COMPONENT(RERUN_EXAMPLE)

class Example : public yarp::os::RFModule {
    public:
        Example() = default;
        virtual ~Example() override = default;

        // //DeviceDriver
        /**
            * Configure with a set of options.
            * @param config The options to use
            * @return true iff the object could be configured.
            */
        // bool open(yarp::os::Searchable& config) override;
        bool configure(yarp::os::ResourceFinder& rf) override;
        bool updateModule() override;
        double getPeriod() override;

    private:
        void loadURDF(rerun::RecordingStream& rr);
        std::string parse_urdf(const std::string& path);
        void extract_joints();
        rerun::RecordingStream rr{"rerun_example", "id_example"};
        std::vector<std::string> axesNames;
        yarp::dev::PolyDriver driver;
        yarp::dev::IEncoders* iEnc{nullptr};
        std::vector<double> encState;
        int axes;
};

#endif // RERUN_EXAMPLE_H