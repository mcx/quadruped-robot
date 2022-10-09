// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

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

#ifndef QR_FOOT_TRAJECTORY_GENERATOR_H
#define QR_FOOT_TRAJECTORY_GENERATOR_H

#include <Eigen/Dense>

#include "common/qr_enums.h"
#include "common/qr_geometry.h"
#include "common/qr_BSpline.h"
#include "../config/config.h"

struct qrStepParameters {
    qrStepParameters() : duration(0.), height(0.), penetration(0.)
    {}
    qrStepParameters(float _duration, float _height, float _penetration = 0.)
        : duration(_duration), height(_height), penetration(_penetration)
    {}

    /** @brief Duration of the step */
    float duration;

    /** @brief Height of the step */
    float height;

    /** @brief Distance of penetration of the swing trajectory */
    float penetration;
};
/**
 * @brief Descripte the spline infomation  
 * @param degree int, the degree of spline, such as 2, 3, 5
 * @param splineType SplineType, option value: quadratic, cubicPolygon, quinticPolygon, BSpline
 */
struct qrSplineInfo {
    int degree=3;
    SplineType splineType = SplineType::CubicPolygon;
    std::vector<glm::vec3> controlPoints;
    std::vector<float> knots;
    qrSplineInfo () {}
    qrSplineInfo (std::vector<glm::vec3> &controlPointsIn, std::vector<float> &knotsIn) {
        splineType = SplineType::BSpline;
        controlPoints = controlPointsIn;
        knots = knotsIn;
    }
};

class qrFootSplinePatternGenerator {
public:
    /** @brief Constructor function */
    qrFootSplinePatternGenerator(): initial_time_(0.), duration_(0.) {};

    /** @brief Destructor function */
    virtual ~qrFootSplinePatternGenerator() = default;

    /**
     * @brief Set the parameters for the generation of the foot swing trajectory
     * This methods assumes that there is not an obstacle in the trajectory.
     * @param const double& Initial time
     * @param const Eigen::Vector3d& Initial foot position
     * @param const Eigen::Vector3d& Target foot position
     * @param const StepParameters Step parameters
     */
    virtual void SetParameters(const float initial_time,
                        const Eigen::Vector3f &initial_pos,
                        const Eigen::Vector3f &target_pos,
                        const qrStepParameters &params) = 0;

    /**
     * @brief Generates the foot-swing trajectory for a given time
     * @param Eigen::Vector3d& Instantaneous foot position
     * @param Eigen::Vector3d& Instantaneous foot velocity
     * @param Eigen::Vector3d& Instantaneous foot acceleration
     * @param const double& Current time
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                            Eigen::Vector3f &foot_vel,
                            Eigen::Vector3f &foot_acc,
                            float time) = 0;
    
    virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos)
    {
        //
    }

protected:
    /** @brief Initial time of the swing trajectory */
    float initial_time_;
    /** @brief Duration of the swing trajectory */
    float duration_;
    Vec3<float> startPos;
    Vec3<float> endPos;
    Mat3<float> R_theta;
    Vec3<float> Tp;
};

class qrFootParabolaPatternGenerator : public qrFootSplinePatternGenerator {
public:
    /** @brief Constructor function */
    qrFootParabolaPatternGenerator();

    /** @brief Destructor function */
    virtual ~qrFootParabolaPatternGenerator();

    /**
     * @brief Set the parameters for the generation of the foot swing trajectory
     * This methods assumes that there is not an obstacle in the trajectory.
     * @param const double& Initial time
     * @param const Eigen::Vector3d& Initial foot position
     * @param const Eigen::Vector3d& Target foot position
     * @param const StepParameters Step parameters
     */
    virtual void SetParameters(const float initial_time,
                        const Eigen::Vector3f &initial_pos,
                        const Eigen::Vector3f &target_pos,
                        const qrStepParameters &params);

    /**
     * @brief Generates the foot-swing trajectory for a given time
     * @param Eigen::Vector3d& Instantaneous foot position
     * @param Eigen::Vector3d& Instantaneous foot velocity
     * @param Eigen::Vector3d& Instantaneous foot acceleration
     * @param const double& Current time
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                            Eigen::Vector3f &foot_vel,
                            Eigen::Vector3f &foot_acc,
                            float time);
    
    virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos)
    {
        //
    }

private:
    /** @brief Spliners for the different axis of the foot movement */
    math::QuadraticSpline foot_spliner_z_;
    qrStepParameters stepParameters;
};


class qrFootCubicPatternGenerator : public qrFootSplinePatternGenerator {
public:
    /** @brief Constructor function */
    qrFootCubicPatternGenerator();

    /** @brief Destructor function */
    virtual ~qrFootCubicPatternGenerator();

    /**
     * @brief Set the parameters for the generation of the foot swing trajectory
     * This methods assumes that there is not an obstacle in the trajectory.
     * @param const double& Initial time
     * @param const Eigen::Vector3d& Initial foot position
     * @param const Eigen::Vector3d& Target foot position
     * @param const StepParameters Step parameters
     */
    virtual void SetParameters(const float initial_time,
                        const Eigen::Vector3f &initial_pos,
                        const Eigen::Vector3f &target_pos,
                        const qrStepParameters &params);

    /**
     * @brief Generates the foot-swing trajectory for a given time
     * @param Eigen::Vector3d& Instantaneous foot position
     * @param Eigen::Vector3d& Instantaneous foot velocity
     * @param Eigen::Vector3d& Instantaneous foot acceleration
     * @param const double& Current time
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                            Eigen::Vector3f &foot_vel,
                            Eigen::Vector3f &foot_acc,
                            float time);
    
    virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos)
    {
        //
    }

private:
    /** @brief Spliners for the different axis of the foot movement */
    math::CubicSpline foot_spliner_x_;
    math::CubicSpline foot_spliner_y_;
    math::CubicSpline foot_spliner_up_z_;
    math::CubicSpline foot_spliner_down_z_;
};

    
class qrFootBSplinePatternGenerator : public qrFootSplinePatternGenerator {
public:
    /** @brief Constructor function */
    qrFootBSplinePatternGenerator(qrSplineInfo &splineInfo);

    /** @brief Destructor function */
    virtual ~qrFootBSplinePatternGenerator() = default;

    virtual void SetParameters(const float initial_time,
                        const Eigen::Vector3f &initial_pos,
                        const Eigen::Vector3f &target_pos,
                        const qrStepParameters &params);
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                    Eigen::Vector3f &foot_vel,
                                    Eigen::Vector3f &foot_acc,
                                    float time);
    virtual void UpdateSpline(float initial_time, float duration, const Eigen::Vector3f &initial_pos, float target_appex,const Eigen::Vector3f &target_pos);

private:
    tinynurbs::Curve3f crv;
    std::vector<glm::vec3> controlPointsTemplate;
};

/**************************************************/
/**************************************************/
class qrSwingFootTrajectory {
public:
    /** brief init func
     * @param Vec3<float> startPosIn
     * @param Vec3<float> endPosIn
     * @param float duration, default value=1.f
     * @param float maxClearance, default value = 0.1f
     */
    qrSwingFootTrajectory() {};

    qrSwingFootTrajectory(qrSplineInfo splineInfoIn,
                        Vec3<float> startPosIn = {0.f, 0.f, 0.f},
                        Vec3<float> endPosIn = {0.f, 0.f, 0.f},
                        float duration = 1.f,
                        float maxClearance = 0.1f
                        );

    qrSwingFootTrajectory(const qrSwingFootTrajectory &item);

    virtual ~qrSwingFootTrajectory() = default;

    /** @brief Call it every time you need a tarjectory point to control. 
     * @param Vec3<float>& foot position;
     * @param Vec3<float>& foot linear velocity;
     * @param Vec3<float>& foot acceleration;
     * @param float time phase, default at range [0,1];
     * @param bool whether phase needs to be moduled, default -<em> false </em>.
     * @return bool flag that indicates whether the process/result/input is correct.
     */
    bool GenerateTrajectoryPoint(Vec3<float> &footPos,
                                    Vec3<float> &footV,
                                    Vec3<float> &footA,
                                    float t,
                                    bool phaseModule=false);

    /** @brief a new cycle begining of the swing foot. */
    void ResetFootTrajectory(float duration, const Vec3<float> &initialPos, const Vec3<float> &targetPos, float height=0.15f)
    {
        stepParams.duration = duration;
        stepParams.height = height;
        footTarjGen->SetParameters(0., initialPos, targetPos, stepParams);
    };

    /** @brief corrupted in the mid air, adjust the behaviar. 
     * todo
     */
    void ResetFootTrajectory(float duration, float currentTime, const Vec3<float> &targetPos)
    {
        stepParams.duration = duration;
        footTarjGen->SetParameters(currentTime, startPos, targetPos, stepParams);
    };

    void Update()
    {
        // footTarjGen->UpdateSpline();
    };

    // private:
    float mid;
    Vec3<float> startPos;
    Vec3<float> endPos;
    qrStepParameters stepParams;
    qrFootSplinePatternGenerator *footTarjGen;
    qrSplineInfo splineInfo;
};
#endif  // QR_FOOT_TRAJECTORY_GENERATOR_H
