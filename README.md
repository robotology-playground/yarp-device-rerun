![YARP logo](https://raw.githubusercontent.com/robotology/yarp/master/doc/images/yarp-robot-24.png "yarp-device-rerun")
yarp-device-rerun
========

This repository contains **`yarpLoggerRerun`**, the [YARP](https://www.yarp.it/) logging device that uses [Rerun.io](https://github.com/rerun-io/rerun) library

# 1. License

[![License](https://img.shields.io/badge/license-BSD--3--Clause%20%2B%20others-19c2d8.svg)](https://github.com/robotology-playground/yarp-device-rerun/blob/master/LICENSE)

This software may be modified and distributed under the terms of the
BSD-3-Clause license. See the accompanying LICENSE file for details.

# 2. Dependencies
Before proceeding further, please install the following dependencies:

- [YARP 3.12](https://github.com/robotology/yarp/releases/tag/v3.12.0)
- [iDynTree](https://robotology.github.io/idyntree)
<!-- - [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) -->

# 3. Install

After installing the necessary dependencies, please choose one of the following methods to build and install the device.

## 3.1 Build from sources

```sh
git clone https://github.com/robotology-playground/yarp-device-rerun
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<installation_path> ..
make
make install
```

This will download a bundle with pre-built Rerun C static libraries for most desktop platforms, all Rerun C++ sources and headers, as well as CMake build instructions for them. See the [dedicated page](https://ref.rerun.io/docs/cpp/md__2home_2runner_2work_2rerun_2rerun_2rerun__cpp_2cmake__setup__in__detail.html) for more details.

In order to make the device detectable, add `<installation_path>/share/yarp` to the `YARP_DATA_DIRS` environment variable of the system.

Alternatively, if `YARP` has been installed using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), it is possible to use `<directory-where-you-downloaded-robotology-superbuild>/build/install` as the `<installation_path>`.

## 4. Usage
This device has to be launched via the yarprobotinterface for logging quantities from the robot (simulated or real) based on what is specified in the configuration file.

The parameters supported so far are:

```
| Parameter name     | Type            | Units   | Default Value    | Required |
|:------------------:|:---------------:|:-------:|:----------------:|:--------:|
| axesNames          | vector<string>  | -       |                  |  Yes     |
| logIEncoders       | bool            | -       |     true         |  No      |
| logIMotorEncoders  | bool            | -       |     false        |  No      |
| logIPidControl     | bool            | -       |     false        |  No      |
| logURDF            | bool            | -       |     false        |  No      |
| fileName           | string          | -       |     log_test     |  No      |
| filePath           | string          | -       |/home/ergocub/test|  No      |
| saveToFile         | bool            | -       |     false        |  No      |
| viewerIp           | string          | -       |    localhost     |  No      |
```

Here an example of xml for using this device on ergoCubSN002:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">

    <device xmlns:xi="http://www.w3.org/2001/XInclude" name="yarpLoggerRerun" type="yarpLoggerRerun">
        <param name="axesNames">(neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_yaw, l_wrist_roll, l_wrist_pitch, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_yaw, r_wrist_roll, r_wrist_pitch, torso_roll, torso_pitch, torso_yaw, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, l_thumb_add, l_thumb_oc, l_index_add, l_index_oc, l_middle_oc, l_ring_pinky_oc, r_thumb_add, r_thumb_oc, r_index_add, r_index_oc, r_middle_oc, r_ring_pinky_oc)</param>
        <param name="logIEncoders">false</param>
        <param name="logIMotorEncoders">false</param>
        <param name="logIPidControl">false</param>
        <param name="logURDF">true</param>
        <param name="fileName">log_test</param>
        <param name="filePath">/home/user/test</param>
        <param name="saveToFile">false</param>
        <param name="viewerIp">localhost</param>

         <action phase="startup" level="5" type="attach">
             <paramlist name="networks">
                <!-- motorcontrol -->
                <elem name="head-j0">head-eb20-j0_2-mc</elem>
                <elem name="head-j2">head-eb21-j3-mc</elem>
                <elem name="torso">torso-eb5-j0_2-mc</elem>
                <elem name="left_upper_arm-j0">left_arm-eb2-j0_1-mc</elem>
                <elem name="left_upper_arm-j2">left_arm-eb4-j2_3-mc</elem>
                <elem name="left_lower_arm">left_arm-eb31-j4_6-mc</elem>
                <elem name="left_hand1">left_arm-eb23-j7_10-mc</elem>
                <elem name="left_hand2">left_arm-eb25-j11_12-mc</elem>
                <elem name="right_upper_arm-j0">right_arm-eb1-j0_1-mc</elem>
                <elem name="right_upper_arm-j2">right_arm-eb3-j2_3-mc</elem>
                <elem name="right_lower_arm">right_arm-eb30-j4_6-mc</elem>
                <elem name="right_hand1">right_arm-eb22-j7_10-mc</elem>
                <elem name="right_hand2">right_arm-eb24-j11_12-mc</elem>
                <elem name="left_upper_leg-j0">left_leg-eb8-j0_3-mc</elem>
                <elem name="left_lower_leg-j2">left_leg-eb9-j4_5-mc</elem>
                <elem name="right_upper_leg-j0">right_leg-eb6-j0_3-mc</elem>
                <elem name="right_lower_leg-j2">right_leg-eb7-j4_5-mc</elem>
             </paramlist>
        </action>
        <action phase="shutdown" level="15" type="detach" />

    </device>
```


Maintainers
--------------
This repository is maintained by:

| | | | |
|:---:|:---:|:---:|:---:|
 [<img src="https://github.com/martinaxgloria.png" width="60">](https://github.com/martinaxgloria) | [@martinaxgloria](https://github.com/martinaxgloria) |<img src="https://user-images.githubusercontent.com/4537987/134487985-e66b9dae-767d-4c3b-9ce1-9e6fb19cf07a.png" width="200" >|

