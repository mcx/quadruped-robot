
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

#include "planner/qr_foothold_planner.h"

qrFootholdPlanner::qrFootholdPlanner(qrRobot *robotIn, qrGaitGenerator* gaitGeneratorIn, qrGroundSurfaceEstimator *groundEsitmatorIn,qrUserParameters* userParametersIn,qrDesiredStateCommand* desiredStateCommandIn)
: robot(robotIn), gaitGenerator(gaitGeneratorIn), groundEsitmator(groundEsitmatorIn), terrain(groundEsitmator->terrain),userParameters(userParametersIn), desiredStateCommand(desiredStateCommandIn),
    timeSinceReset(0.f)
{
    footstepper = new qrFootStepper(terrain, 0.10f, "optimal");
    Reset(0);
}

void qrFootholdPlanner::Reset(float t)
{
    resetTime = t;
    timeSinceReset = 0.f;
    footstepper->Reset(timeSinceReset);
    comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
    desiredComPose = Eigen::Matrix<float, 6, 1>::Zero();
    desiredFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
    if (robot->locomotionMode == LocomotionMode::ADVANCED_TROT_LOCOMOTION) {
        swingKp = Eigen::MatrixXf::Map(&userParameters->swingKp["advanced_trot"][0], 3, 1);
    } else {
        swingKp = Eigen::MatrixXf::Map(&userParameters->swingKp["trot"][0], 3, 1);
    }
    firstSwingBaseState << robot->GetBasePosition(), robot->stateDataFlow.baseVInWorldFrame, 
                            robot->GetBaseRollPitchYaw(), robot->GetBaseRollPitchYawRate();
}

void qrFootholdPlanner::UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, std::vector<int> legIds)
{
    comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
    desiredComPose << 0.f,0.f,0.f,0.f,0.f,0.f; //comPose;
    desiredFootholds = currentFootholds;
    if (legIds.empty()) { // if is empty, update all legs.
        legIds = {0,1,2,3};
    } else {
        std::cout<<"update foothold of Legs : ";
        for(int legId : legIds) {
            std::cout << legId << " ";
        }
        std::cout << "\n";
    }
    
    if (terrain.terrainType != TerrainType::STAIRS) { 
        ComputeFootholdsOffset(currentFootholds, comPose, desiredComPose, legIds);
    } else {
        ComputeNextFootholds(currentFootholds, comPose, desiredComPose, legIds);
    }
}

Eigen::Matrix<float, 3, 4> qrFootholdPlanner::ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                                Eigen::Matrix<float, 6, 1> currentComPose,
                                                                Eigen::Matrix<float, 6, 1> desiredComPose,
                                                                std::vector<int> legIds)
{
    desiredFootholdsOffset = footstepper->GetOptimalFootholdsOffset(currentFootholds);
    return desiredFootholdsOffset;
}

Eigen::Matrix<float, 3, 4> qrFootholdPlanner::ComputeNextFootholds(Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                                    Eigen::Matrix<float, 6, 1>& currentComPose,
                                                                    Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                                    std::vector<int>& legIds)
{
        
    auto res = footstepper->GetFootholdsInWorldFrame(currentFootholds, currentComPose, desiredComPose, legIds);
    desiredFootholds = std::get<0>(res);
    desiredFootholdsOffset = std::get<1>(res);
    return desiredFootholdsOffset;
}

 void qrFootholdPlanner::ComputeHeuristicFootHold(std::vector<u8> swingFootIds)
{
        if (swingFootIds.empty()) {
            return;
        }
        Vec3<float> desiredSpeed = desiredStateCommand->stateDes.segment(6, 3);
        float desiredTwistingSpeed = desiredStateCommand->stateDes(11);
        Vec3<float> desiredHeight(0.f,0.f, desiredStateCommand->stateDes(2) - userParameters->footClearance);
        
        Eigen::Matrix<float,3,4> footPositionsInBaseFrame = robot->state.GetFootPositionsInBaseFrame();
        Eigen::Matrix<float,3,4> footPositionsInWorldFrame = robot->state.GetFootPositionsInWorldFrame();
        
        Mat3<float> robotBaseR = robot->stateDataFlow.baseRMat;
        Mat3<float> controlFrameR = robot->stateDataFlow.groundRMat;
        Mat3<float> dR = robot->stateDataFlow.baseRInControlFrame;
        Eigen::Matrix<float, 12, 1> currentJointAngles = robot->GetMotorAngles();
        Vec3<float> comVelocity = robot->GetBaseVelocity(); // in base frame
        Vec3<float> w = robot->GetBaseRollPitchYawRate();
        float yawDot = w[2]; // in base frame
        Eigen::Matrix<float,3,4> hipPositions = robot->GetHipPositionsInBaseFrame();
        Eigen::Matrix<float, 3, 4> abadPosInBaseFrame = robot->GetHipOffset();
        
        for (u8& legId : swingFootIds) {
            Vec3<float> hipOffset = abadPosInBaseFrame.col(legId); // hipPositions.col(legId);
            Vec3<float> twistingVector = {-hipOffset[1], hipOffset[0], 0.f};
            Vec3<float> hipHorizontalVelocity = comVelocity + w.cross(hipOffset); // yawDot * twistingVector; // in base frame

            hipHorizontalVelocity = dR * hipHorizontalVelocity; // in control frame
            hipHorizontalVelocity[2] = 0.f;
            Vec3<float> targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector; // in control frame
         
            Vec3<float> footTargetPosition;
            Vec3<float> footTargetHipPosition;
            
            float hipLen = 0.085;
            float side_sign[4] = {-1, 1, -1, 1}; // y-axis
            // float abad = currentJointAngles[3*legId+0];
            float abad = -robot->GetBaseRollPitchYaw()[0];
            footTargetHipPosition[0] = abadPosInBaseFrame(0, legId);
            footTargetHipPosition[1] = abadPosInBaseFrame(1, legId) + side_sign[legId]*hipLen * cos(abad);
            footTargetHipPosition[2] = abadPosInBaseFrame(2, legId) + side_sign[legId]*hipLen * sin(abad);
            // footTargetHipPosition = robotBaseR*footTargetHipPosition;  // in translated world frame;
            Vec3<float> angles = currentJointAngles.segment(3*legId,3);
            angles[0] = 0;
            Vec3<float> footPosInHipFrame = robot->config->FootPositionInHipFrame(angles,pow((-1), legId + 1));
            
            // todo: add switch gait code to here
            moveDown[legId] = 0;
            Vec3<float> dP;
            float swingRemainTime = gaitGenerator->swingTimeRemaining[legId];
            float rollCorrect = w[0] * 0.3f * swingRemainTime;
            float pitchCorrect = -w[1] * 0.3f * swingRemainTime;
            if (robot->locomotionMode ==LocomotionMode::ADVANCED_TROT_LOCOMOTION && false) {
                dP = dR.transpose() * (targetHipHorizontalVelocity - hipHorizontalVelocity) * gaitGenerator->stanceDuration[legId] / 2.0
                        - swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity);
            } else {
                
                dP = dR.transpose() * (targetHipHorizontalVelocity * swingRemainTime
                        - swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity));
            }
            const float dPthresold = 0.2f;
            
            dP = dP.cwiseMin(dPthresold).cwiseMax(-dPthresold); // clip
            if (dP[1] > 0.15) {
                dP[1] = 0.15;
            } else if (dP[1] < -0.15) {
                dP[1] = -0.15;
            }
            dP[2] = 0;
            Vec3<float> rpy = robot->GetBaseRollPitchYaw();
            float interleave_y[4] = {-0.08, 0.08, -0.08, 0.08};
            Mat3<float> rollR = math::coordinateRotation(math::CoordinateAxis::X, rpy[0]); // abad->hip offset vector
        
            footTargetPosition = dP
                    + Vec3<float>(hipOffset[0], hipOffset[1], 0)
                    + rollR * Vec3<float>{0,interleave_y[legId],0};
            if (desiredStateCommand->stateDes(6,0)< -0.01) {
                footTargetPosition(0,2) -= 0.02;
                footTargetPosition(0,3) -= 0.02;
            }

            footTargetPosition -=  robotBaseR.transpose() * desiredHeight;
            phase[legId] = gaitGenerator->normalizedPhase(legId);

        desiredFootholds.col(legId) = footTargetPosition; // in base frame       
    }
}
