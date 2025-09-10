#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/DeviceDriver.h>

#include "Example.h"

using namespace std;

int main(int argc, char *argv[]) {

    yarp::os::Network yarp;
    if(!yarp.checkNetwork())
    {
        yError() << "Yarp doesn't work!";
        return EXIT_FAILURE;
    }

    Example example;
    yDebug() << "Starting example module...";
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    return example.runModule(rf);
}