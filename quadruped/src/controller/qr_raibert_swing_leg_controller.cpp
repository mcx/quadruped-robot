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

#include "controller/qr_raibert_swing_leg_controller.h"

using namespace Eigen;
using namespace std;

float qrSwingLegController::GenParabola(float phase, float start, float mid, float end)
{
    float mid_phase = 0.5;
    float deltaOne, deltaTwo, deltaThree, coefa, coefb, coefc;
    deltaOne = mid - start;
    deltaTwo = end - start;
    deltaThree = pow(mid_phase, 2) - mid_phase;
    coefa = (deltaOne - deltaTwo * mid_phase) / deltaThree;
    coefb = (deltaTwo * pow(mid_phase, 2) - deltaOne) / deltaThree;
    coefc = start;
    return coefa * pow(phase, 2) + coefb * phase + coefc;
}

Matrix<float, 3, 1> qrSwingLegController::GenSwingFootTrajectory(float inputPhase,
                                                                        Matrix<float, 3, 1> startPos,
                                                                        Matrix<float, 3, 1> endPos)
{
    float phase = inputPhase;
    if (inputPhase <= 0.5) {
        phase = 0.8 * sin(inputPhase * M_PI);
    } else {
        phase = 0.8 + (inputPhase - 0.5) * 0.4;
    }
    float x, y, maxClearance, mid, z;
    x = (1 - phase) * startPos(0, 0) + phase * endPos(0, 0);
    y = (1 - phase) * startPos(1, 0) + phase * endPos(1, 0);
    maxClearance = 0.1;
    mid = max(endPos(2, 0), startPos(2, 0)) + maxClearance;
    z = GenParabola(phase, startPos(2, 0), mid, endPos(2, 0));
    return Matrix<float, 3, 1>(x, y, z);
}

qrSwingLegController::qrSwingLegController(qrRobot *robot,
                                            qrGaitGenerator *gaitGenerator,
                                            qrRobotEstimator *stateEstimator,
                                            qrGroundSurfaceEstimator *groundEstimator,
                                            qrFootholdPlanner *footholdPlanner,
                                            qrUserParameters *userParameters,
                                            Matrix<float, 3, 1> desiredSpeed,
                                            float desiredTwistingSpeed,
                                            float desiredHeight,
                                            float footClearance,
                                            std::string configPath)
{
    this->robot = robot;
    this->gaitGenerator = gaitGenerator;
    this->stateEstimator = stateEstimator;
    this->groundEstimator = groundEstimator;
    this->footholdPlanner = footholdPlanner;
    this->desiredSpeed = desiredSpeed;
    this->desiredTwistingSpeed = desiredTwistingSpeed;
    this->desiredHeight = Matrix<float, 3, 1>(0, 0, desiredHeight - footClearance);
    this->swingKp = Matrix<float, 3, 1>(0.03, 0.03, 0.03);
    this->userParameters = userParameters;
    swingLegConfig = YAML::LoadFile(configPath);
    footInitPose = swingLegConfig["swing_leg_params"]["foot_in_world"].as<std::vector<std::vector<float>>>();
    footOffset = swingLegConfig["swing_leg_params"]["foot_offset"].as<float>();
}

void qrSwingLegController::Reset(float currentTime)
{ 
    phaseSwitchFootLocalPos = robot->state.GetFootPositionsInBaseFrame();
    desiredFootPositionsInBaseFrame = phaseSwitchFootLocalPos;
    
    foot_pos_rel_last_time = phaseSwitchFootLocalPos;
    foot_pos_target_last_time = foot_pos_rel_last_time;
    foot_vel_error.setZero();
    foot_pos_error.setZero();

    phaseSwitchFootGlobalPos = robot->state.GetFootPositionsInWorldFrame();
    desiredStateCommand->footTargetPositionsInWorldFrame = phaseSwitchFootGlobalPos;
    footHoldInControlFrame = phaseSwitchFootLocalPos;

    footholdPlanner->Reset(currentTime); // reset planner

    switch (robot->locomotionMode) 
    {
    case LocomotionMode::POSITION_LOCOMOTION: {
        footHoldInWorldFrame = phaseSwitchFootGlobalPos;
        footHoldInWorldFrame(0, 0) -= 0.05;
        footHoldInWorldFrame(0, 3) -= 0.05;
    }break;
    case LocomotionMode::ADVANCED_TROT_LOCOMOTION: {
        swingKp << 0.01,0.03,0.01;
    }
    default:
        break;
    }

    splineInfo.splineType = SplineType::XYLinear_ZParabola;
    for (int i = 0; i < NumLeg; ++i) {
        swingFootTrajectories[i] = qrSwingFootTrajectory(splineInfo, phaseSwitchFootLocalPos.col(i),                                                        
            phaseSwitchFootLocalPos.col(i), 1.f, 0.15);
    }
    swingJointAnglesVelocities.clear();
}

void qrSwingLegController::Update(float currentTime)
{
    const Vec4<int>& newLegState = gaitGenerator->desiredLegState;
    const Vec4<int>& curLegState = gaitGenerator->curLegState;
    // the footHoldOffset is first initialized at qr_robot.h, then update it at qr_ground_estimator.cpp 
    Eigen::Matrix<float, 3, 1> constOffset = {robot->config->footHoldOffset, 0.f, 0.f};
    Quat<float> robotComOrientation = robot->GetBaseOrientation();
    Vec3<float> robotComRPY = robot->GetBaseRollPitchYaw();
    Mat3<float> Rb = robot->stateDataFlow.baseRMat;
    Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
    Mat3<float> RcSource = robot->stateDataFlow.groundRMat;
    Mat3<float> Rcb = robot->stateDataFlow.baseRInControlFrame;
    swingFootIds.clear(); 
    /* 
        detects phase switch for each leg so we can remember the feet position at
        the beginning of the swing phase.
    */
    switch (robot->locomotionMode) {
        case LocomotionMode::VELOCITY_LOCOMOTION: {
            for (int legId = 0; legId < NumLeg; ++legId) {
                if (newLegState(legId) == LegState::SWING && newLegState(legId) != gaitGenerator->curLegState(legId)) {
                    phaseSwitchFootLocalPos.col(legId) = robot->state.GetFootPositionsInBaseFrame().col(legId);
                    phaseSwitchFootGlobalPos.col(legId) = Rb * phaseSwitchFootLocalPos.col(legId); // robot->GetFootPositionsInWorldFrame().col(legId);
                    if (robot->locomotionMode ==LocomotionMode::ADVANCED_TROT_LOCOMOTION) {
                        splineInfo.splineType = SplineType::BSpline; // BSpline, XYLinear_ZParabola
                        footholdPlanner->firstSwingBaseState << robot->GetBasePosition(), robot->stateDataFlow.baseVInWorldFrame,
                                robot->GetBaseRollPitchYaw(), robot->GetBaseRollPitchYawRate();
                    } else {
                        splineInfo.splineType = SplineType::XYLinear_ZParabola; // BSpline, XYLinear_ZParabola
                    }
                    swingFootTrajectories[legId] = qrSwingFootTrajectory(splineInfo, phaseSwitchFootLocalPos.col(legId), phaseSwitchFootLocalPos.col(legId), 1.f, 0.15);
                }
            }
        }break;
        case LocomotionMode::POSITION_LOCOMOTION: {
            for (int legId = 0; legId < NumLeg; ++legId) {
                if ((newLegState(legId) == LegState::SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                    && curLegState(legId) == LegState::STANCE 
                    && !robot->stop) {
        
                    phaseSwitchFootLocalPos.col(legId) = robot->state.GetFootPositionsInBaseFrame().col(legId);
                    phaseSwitchFootGlobalPos.col(legId) = robot->state.GetFootPositionsInWorldFrame().col(legId);
                    // update four legs' footholds
                    if (legId == 0) { 
                        // based on the last foothold position
                        footholdPlanner->UpdateOnce(footHoldInWorldFrame); 
                    }
                    footHoldInWorldFrame.col(legId) += footholdPlanner->GetFootholdsOffset().col(legId);
                }
            }
        }break;
        default:
            break;
    }

    for (u8 legId(0); legId < NumLeg; ++legId) {
        int tempState = gaitGenerator->legState[legId];
        
        if ((tempState == LegState::STANCE) || tempState == LegState::EARLY_CONTACT) {
            continue;
        } else {
            swingFootIds.push_back(legId);
        }
    }

    if (robot->locomotionMode == LocomotionMode::ADVANCED_TROT_LOCOMOTION) {
            footholdPlanner->ComputeHeuristicFootHold(swingFootIds);
    }
}

// Controls the swing leg position using Raibert's formula.
// For details, please refer to chapter 2 in "Legged robbots that balance" byMarc Raibert. 
// The key idea is to stablize the swing foot's location based onthe CoM moving speed.
// see the following paper for details:https://ieeexplore.ieee.org/document/8593885

void qrSwingLegController::VelocityLocomotionProcess(const Matrix<float, 3, 3> &dR,
                                                    const Mat3<float> &robotBaseR, 
                                                    Matrix<float, 3, 1> &footPositionInBaseFrame,
                                                    Matrix<float, 3, 1> &footVelocityInBaseFrame,
                                                    Matrix<float, 3, 1> &footAccInBaseFrame, 
                                                    int legId)
{
    Matrix<float, 3, 1> baseVelocity;
    Matrix<float, 3, 1> hipOffset;
    Matrix<float, 3, 1> twistingVector;
    Matrix<float, 3, 1> footTargetPosition;
    Matrix<float, 3, 1> hipHorizontalVelocity;
    Matrix<float, 3, 1> targetHipHorizontalVelocity;
    Matrix<float, 3, 4> hipPositions;
    float yawDot;

    hipPositions = robot->GetHipPositionsInBaseFrame();
    yawDot = robot->GetBaseRollPitchYawRate()(2, 0);
    hipOffset = hipPositions.col(legId);
    baseVelocity = stateEstimator->GetEstimatedVelocity();

    twistingVector = Matrix<float, 3, 1>(-hipOffset[1], hipOffset[0], 0);
    // hipHorizontal velocity in base frame
    hipHorizontalVelocity = baseVelocity + yawDot * twistingVector; 
    // hipHorizontal velocity in control frame
    hipHorizontalVelocity = dR * hipHorizontalVelocity; 
    hipHorizontalVelocity[2] = 0.f;
    // targetHipHorizontal velocity In control frame
    // targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector;
    targetHipHorizontalVelocity = desiredStateCommand->stateDes.segment(6,3) + desiredStateCommand->stateDes(11) * twistingVector; // in control frame
        
    footTargetPosition = dR.transpose() * (hipHorizontalVelocity * gaitGenerator->stanceDuration[legId] / 2.0 -
        swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity))
            + Matrix<float, 3, 1>(hipOffset[0], hipOffset[1], 0)
            -  robotBaseR.transpose() * desiredHeight;

    swingFootTrajectories[legId].ResetFootTrajectory(1.f, phaseSwitchFootLocalPos.col(legId), footTargetPosition, 0.1);
    bool flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInBaseFrame,
                                                                                    footVelocityInBaseFrame,
                                                                                    footAccInBaseFrame,
                                                                                    gaitGenerator->normalizedPhase(legId),
                                                                                    true);
}

void qrSwingLegController::PositionLocomotionProcess(Matrix<float, 3, 1> &footPositionInWorldFrame, 
                                                    Matrix<float, 3, 1> &footPositionInBaseFrame,
                                                    Matrix<float, 3, 1> &footVelocityInWorldFrame,
                                                    Matrix<float, 3, 1> &footAccInWorldFrame, 
                                                    int legId)
{
    // interpolation in world frame
     swingFootTrajectories[legId].ResetFootTrajectory(1.f, phaseSwitchFootGlobalPos.col(legId), footHoldInWorldFrame.col(legId), 0.1);
                    bool flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInWorldFrame,
                                                                                    footVelocityInWorldFrame,
                                                                                    footAccInWorldFrame,
                                                                                    gaitGenerator->normalizedPhase(legId),
                                                                                    true); 
    // transfer foot position to base frame
    footPositionInBaseFrame = math::RigidTransform(robot->GetBasePosition(),
                                                    robot->GetBaseOrientation(),
                                                   footPositionInWorldFrame);
}
void qrSwingLegController::AdvancedLocomotionProcess(qrStateDataFlow &stateData,
                                    Eigen::Matrix<float, 3, 1> &footPositionInWorldFrame, 
                                    Eigen::Matrix<float, 3, 1> &footPositionInBaseFrame,
                                    Eigen::Matrix<float, 3, 1> &footVelocityInBaseFrame,
                                    Eigen::Matrix<float, 3, 1> &footVelocityInWorldFrame,
                                    Eigen::Matrix<float, 3, 1> &footAccInBaseFrame,
                                    Eigen::Matrix<float, 3, 1> &footAccInWorldFrame,
                                    const Mat3<float> &robotBaseR,
                                    Quat<float> &robotComOrientation,
                                    int legId)
{
    Eigen::Matrix<float,3,4> footVTarget;
    Eigen::Matrix<float,3,4> footVCurrent;    
    Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame = robot->state.GetFootPositionsInBaseFrame();
    Eigen::Matrix<float, 3, 4> footVelocitysInBaseFrame = robot->stateDataFlow.footVelocitiesInBaseFrame;
    Eigen::Matrix<float, 3, 1> footTargetPosition;
    float phase;
    footTargetPosition = footholdPlanner->desiredFootholds.col(legId);
    phase = footholdPlanner->phase[legId];

    float H = 0.1;
    if (desiredStateCommand->stateDes(6, 0) > 0.01) {
        H = robot->config->lowerLegLength * 0.8; // 0.2*0.8 = 0.16(a1), 0.25*0.8=0.2(aliengo)
    } else if (desiredStateCommand->stateDes(6, 0) < -0.01){
        H = 0.1;
    } else {
        H = robot->config->lowerLegLength * 0.6; // 0.2 *0.6 = 0.12(a1), 0.15(aliengo)
    }

    Vec3<float> footTargetPositionInWorldFrame = math::invertRigidTransform(robot->GetBasePosition(), robotComOrientation, footTargetPosition);
    desiredStateCommand->footTargetPositionsInWorldFrame.col(legId) = footTargetPositionInWorldFrame;
    
    swingFootTrajectories[legId].ResetFootTrajectory(1.f, phaseSwitchFootGlobalPos.col(legId), robotBaseR*footTargetPosition, H);
    bool flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInWorldFrame,
                                                                    footVelocityInWorldFrame,
                                                                    footAccInWorldFrame,
                                                                    phase,
                                                                    false);

    footPositionInBaseFrame = robotBaseR.transpose()* footPositionInWorldFrame;
    desiredFootPositionsInBaseFrame.col(legId) = footPositionInBaseFrame;
    if (phase < 1.0) {
        footVelocityInBaseFrame = robotBaseR.transpose()*footVelocityInWorldFrame;
        footVelocityInBaseFrame /= gaitGenerator->swingDuration[legId];
    }    
    Vec3<float> foot_vel_cur = footVelocitysInBaseFrame.col(legId);//(footPositionsInBaseFrame.col(legId) - foot_pos_rel_last_time.col(legId)) / 0.001;
    foot_pos_rel_last_time.col(legId) = footPositionsInBaseFrame.col(legId);

    Vec3<float> foot_vel_target = footVelocityInBaseFrame;//(footPositionInBaseFrame - foot_pos_target_last_time.col(legId)) / 0.002;
    foot_pos_target_last_time.col(legId) = footPositionInBaseFrame;

    foot_pos_error.col(legId) = footPositionInBaseFrame - footPositionsInBaseFrame.col(legId);
    foot_vel_error.col(legId) = foot_vel_target - foot_vel_cur;
    footVTarget.col(legId) = foot_vel_target;
    footVCurrent.col(legId) = foot_vel_cur;
    
    stateData.wbcData.pFoot_des[legId] = math::invertRigidTransform(robot->GetBasePosition(), robotComOrientation, footPositionInBaseFrame); // in world Frame
    stateData.wbcData.vFoot_des[legId] = stateData.baseVInWorldFrame + robotBaseR * footVelocityInBaseFrame;
    stateData.wbcData.aFoot_des[legId] = robotBaseR * footAccInBaseFrame;
}
void qrSwingLegController::UpdateControlParameters(const Vector3f& linSpeed, const float& angSpeed)
{
    desiredSpeed = linSpeed;
    desiredTwistingSpeed = angSpeed;
}

map<int, Matrix<float, 5, 1>> qrSwingLegController::GetAction()
{
    auto& stateData = robot->stateDataFlow;
    Matrix<float, 3, 1> footPositionInBaseFrame, footVelocityInBaseFrame, footAccInBaseFrame;
    Matrix<float, 3, 1> footPositionInWorldFrame, footVelocityInWorldFrame, footAccInWorldFrame;
    Matrix<float, 3, 1> footPositionInControlFrame, footVelocityInControlFrame, footAccInControlFrame;
    Matrix<int, 3, 1> jointIdx;
    Matrix<float, 3, 1> jointAngles;
    Matrix<float, 3, 4> hipPositions;
    Matrix<float, 12, 1> currentJointAngles = robot->GetMotorAngles();

    Quat<float> robotComOrientation = robot->GetBaseOrientation();
    Mat3<float> robotBaseR = math::quaternionToRotationMatrix(robotComOrientation).transpose();
    Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
    Mat3<float> dR; // represent base frame in control frame
    if (groundEstimator->terrain.terrainType < 2) {
        dR = Mat3<float>::Identity();
        robotBaseR = Mat3<float>::Identity();
    } else {
        dR = math::quaternionToRotationMatrix(controlFrameOrientation) * robotBaseR;
    }


    for (u8& legId : swingFootIds) {

        footVelocityInBaseFrame.setZero();
        footVelocityInWorldFrame.setZero();
        footVelocityInControlFrame.setZero();
        footAccInBaseFrame.setZero();
        footAccInWorldFrame.setZero();
        footAccInControlFrame.setZero();
        
        switch (robot->locomotionMode) {
            case LocomotionMode::VELOCITY_LOCOMOTION:
                VelocityLocomotionProcess(dR,robotBaseR, footPositionInBaseFrame, footVelocityInBaseFrame,footAccInBaseFrame, legId);
                break;
            case LocomotionMode::POSITION_LOCOMOTION:
                PositionLocomotionProcess(footPositionInWorldFrame, footPositionInBaseFrame, footVelocityInWorldFrame, footAccInWorldFrame, legId);
                break;
            case LocomotionMode::ADVANCED_TROT_LOCOMOTION:
                AdvancedLocomotionProcess(stateData,footPositionInWorldFrame,footPositionInBaseFrame,footVelocityInBaseFrame,footVelocityInWorldFrame, footAccInBaseFrame, footAccInWorldFrame,robotBaseR, robotComOrientation,legId);
            default:
                break;
        }
        // compute joint position & joint velocity
        robot->config->ComputeMotorAnglesFromFootLocalPosition(legId, footPositionInBaseFrame, jointIdx, jointAngles);
        Vec3<float> motorVelocity = robot->config->ComputeMotorVelocityFromFootLocalVelocity(
            legId, jointAngles, footVelocityInBaseFrame);
        // check nan value
        int invalidAngleNum = 0;
        for (int i = 0; i < numMotorOfOneLeg; ++i) {
            if(isnan(jointAngles[i])) {
                invalidAngleNum++;
                jointAngles[i] = currentJointAngles[numMotorOfOneLeg* legId + i];
            }
            swingJointAnglesVelocities[jointIdx[i]] = {jointAngles[i], motorVelocity[i], legId};
        }
    }
    map<int, Matrix<float, 5, 1>> actions;
    Matrix<float, 12, 1> kps, kds;
    kps = robot->config->motorKps;
    kds = robot->config->motorKds;

    // force contrl by PD computation
    for (int singleLegId(0); singleLegId<NumLeg; ++singleLegId) {
        bool flag;
        flag = (gaitGenerator->legState[singleLegId] == LegState::SWING) // for trot up to slope,  // in velocity mode
                    || (gaitGenerator->curLegState[singleLegId] == LegState::SWING); // for gait schedule mode 
        if (flag) {
            foot_forces_kin.col(singleLegId) = foot_pos_error.block<3, 1>(0, singleLegId).cwiseProduct(kps.segment(3*singleLegId,3))/4.0 + // 40, 4
                                            foot_vel_error.block<3, 1>(0, singleLegId).cwiseProduct(kds.segment(3*singleLegId,3))/1.0; // 5, 10, 2
            Mat3<float> jac = stateData.footJvs[singleLegId];
            Vec3<float> joint_torques = jac.lu().solve(foot_forces_kin.col(singleLegId));
            
            joint_torques = joint_torques.cwiseMax(-20).cwiseMin(20);
            actions[3*singleLegId] << 0, 0, 0, 0, joint_torques[0];
            actions[3*singleLegId+1] << 0, 0, 0, 0, joint_torques[1];
            actions[3*singleLegId+2] << 0, 0, 0, 0, joint_torques[2];
        
        }
    }
    for (auto it = swingJointAnglesVelocities.begin(); it != swingJointAnglesVelocities.end(); ++it) {
        const std::tuple<float, float, int> &posVelId = it->second;
        const int singleLegId = std::get<2>(posVelId);
        
        bool flag;
        switch (robot->locomotionMode) {
            case LocomotionMode::VELOCITY_LOCOMOTION: {
                flag = (gaitGenerator->desiredLegState[singleLegId] == LegState::SWING);
            } break;
            case LocomotionMode::POSITION_LOCOMOTION: {
                flag = (gaitGenerator->legState[singleLegId] == LegState::SWING);
            } break;
            default: {
                flag = (gaitGenerator->legState[singleLegId] == LegState::SWING) // for trot up to slope,  // in velocity mode
                            || (gaitGenerator->curLegState[singleLegId] == LegState::SWING); // for gait schedule mode 
            } break;
        }
        if (flag) {
            actions[it->first] << std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId), kds[it->first], actions[it->first][4];
        }
    } 
    return actions;
}
