// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_DESIRED_STATE_COMMAND_H
#define QR_DESIRED_STATE_COMMAND_H

#include "config.h"
#include "common/qr_eigen_types.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "robots/qr_robot.h"


class qrDesiredStateCommand {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    qrDesiredStateCommand(ros::NodeHandle &nhIn, qrRobot*);

    ~qrDesiredStateCommand() = default;

    void Update();
    void SetCommandLimits(float minVelX_in, float maxVelX_in,
                            float minVelY_in, float maxVelY_in, 
                            float minTurnRate_in, float maxTurnRate_in);
    void DesiredStateTrajectory(int N, Vec10<float> dtVec);
    void PrintRawInfo();
    void PrintStateCommandInfo();
    float Deadband(float command, const float &minVal, const float &maxVal);
    void JoyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    const float MAX_ROLL = 0.4;
    const float MIN_ROLL = -0.4;
    const float MAX_PITCH = 0.5;
    const float MIN_PITCH = -0.5;
    const float MAX_VELX = 0.2;
    const float MIN_VELX = -0.15;
    const float MAX_VELY = 0.1; // 0.05
    const float MIN_VELY = -0.1;
    const float MAX_YAWRATE = 0.2;
    const float MIN_YAWRATE = -0.2;
    const float MAX_ROLLRATE = 0.2;
    const float MIN_ROLLRATE = -0.2;
    const float MAX_PITCHRATE = 0.2;
    const float MIN_PITCHRATE = -0.2;
    const float BODY_HEIGHT_MAX = 0.5;
    const float BODY_HEIGHT_MIN = 0.15;

    Vec3<float> vDesInBodyFrame;
    Vec3<float> wDesInBodyFrame;
    // Holds the instantaneous desired state and future desired state trajectory
    Vec12<float> stateDes;
    Vec12<float> stateCur;
    Vec12<float> preStateDes;
    Vec18<float> ddqDes;// base(6, ax ay az,dwx,dwy,dwz) + joint(12)
    Eigen::Matrix<float, 3, 4> legJointq; // control joint q
    Eigen::Matrix<float, 3, 4> legJointdq; // control joint dq
    Eigen::Matrix<float, 12, 10> stateTrajDes; // history states
    Eigen::Matrix<float, 3, 4> footTargetPositionsInWorldFrame;
    int movementMode;
    float joycmdBodyHeight;
    float joyCmdVz, joyCmdVx, joyCmdVy;
    float joyCmdYawRate, joyCmdRollRate, joyCmdPitchRate;
    bool joyCtrlStateChangeRequest;
    bool joyCtrlOnRequest;
    bool rosCmdRequest;
    bool joyCmdExit;
    int bodyUp;
    Vec3<float> filteredVel;
    Vec3<float> filteredOmega;
    Vec3<float> rootPosDes;
    RC_MODE JoyCtrlState, prevJoyCtrlState;
    ros::Subscriber gamepadCommandSub;
    std::string topicName = "/joy";
    bool gaitSwitch = false;
    long long count =0;
private:
    // Dynamics matrix for discrete time approximation
    ros::NodeHandle &nh;
    Mat12<float> A;

    // Control loop timestep change
    float dt = 0.002;

    // Value cutoff for the analog stick Deadband
    const float deadbandRegion = 1;
    //const T filter = 0.01;
    const float filterFactor = 0.02;

    // Choose how often to print info, every N iterations
    int printNum = 5;// N*(0.001s) in simulation time
    // Track the number of iterations since last info print
    int printIter = 0;
};
#endif // QR_DESIRED_STATE_COMMAND_H
