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

#ifndef QR_STATE_DATAFLOW_H
#define QR_STATE_DATAFLOW_H

#include <yaml-cpp/yaml.h>
#include "common/qr_se3.h"
#include "common/qr_geometry.h"
#include "../config/config.h"


struct qrUserParameters {
    qrUserParameters(std::string filePath);
    float stairsTime; 
    float stairsVel;
    unsigned int controlFrequency = 500; // hz
    
    // ground estimator
    unsigned int filterWindowSize = 50;

    // velocity estimator
    float accelerometerVariance = 0.01f; // 0.1f
    float sensorVariance = 0.01f;
    float initialVariance = 0.01f;
    int movingWindowFilterSize = 50; // 120 ms
    
    // swing controller
    float desiredHeight = A1_BODY_HIGHT;
    Vec3<float> desiredSpeed = {0.f, 0.f, 0.f};
    float desiredTwistingSpeed = 0.f;
    float footClearance = 0.01f;
    Vec4<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45}; // FR, FL, RR, RL
    std::map<std::string, std::vector<float>> swingKp;

    // stance controller
    bool computeForceInWorldFrame = true;
    // mit stance leg controller
    bool useWBC = true;

};


// template<typename T>
class WbcCtrlData {
  public:
    Vec3<float> pBody_des;
    Vec3<float> vBody_des;
    Vec3<float> aBody_des;
    Vec3<float> pBody_RPY_des;
    Vec3<float> vBody_Ori_des;

    Vec3<float> pFoot_des[4];
    Vec3<float> vFoot_des[4];
    Vec3<float> aFoot_des[4];
    Vec3<float> Fr_des[4];

    Vec4<bool> contact_state;

    bool allowAfterMPC = true;
};

struct qrStateDataFlow
{
    Eigen::Matrix<float,3,4> footPositionsInBaseFrame;
    Eigen::Matrix<float, 3, 4> footVelocitiesInBaseFrame;
    Vec3<float> baseVInWorldFrame; // in World frame 
    Vec3<float> baseWInWorldFrame; //BASE Angular velocity in World frame
    Vec3<float> baseLinearAcceleration; // in base/IMU frame 
    std::vector<Mat3<float>> footJvs; // Jacobian
    Eigen::Matrix<float, 3, 4> estimatedFootForce; // in base frame
    Vec3<float> estimatedMoment; // in base frame
    float heightInControlFrame = 0.27;
    Vec3<float> zmp;
    
    Mat3<float> baseRMat;
    Mat3<float> groundRMat;
    Vec4<float> groundOrientation;
    Mat3<float> baseRInControlFrame;
    WbcCtrlData wbcData;

    qrStateDataFlow()
    {
        footPositionsInBaseFrame.setZero();
        footVelocitiesInBaseFrame.setZero();
        baseVInWorldFrame.setZero();
        baseWInWorldFrame.setZero();
        baseLinearAcceleration.setZero();
        footJvs = std::vector<Mat3<float>>(4, Mat3<float>::Identity());
        estimatedFootForce.setZero();
        estimatedMoment.setZero();
        zmp.setZero();
        baseRMat.setIdentity();
        groundRMat.setIdentity();
        groundOrientation << 1.f, 0.f, 0.f, 0.f;
        baseRInControlFrame.setIdentity();

    }       
};

#endif //QR_STATE_DATAFLOW_H
