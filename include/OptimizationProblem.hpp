#pragma once

#include "Pose.hpp"
#include "ImagePointContainer.hpp"
#include "ObjectPointContainer.hpp"
#include "CameraParametersContainer.hpp"
#include "ReprojectionError.hpp"
#include "ObjectPointError.hpp"
#include "PoseCovariance.hpp"

#include <ceres/problem.h>

#include <unordered_set>

using namespace std::literals;

namespace multicamera_resection
{

enum class OptimizationStatus {SUCCESS, NOT_ENOUGH_MEASUREMENTS, NOT_USABLE};

static const std::map<OptimizationStatus, std::string> optimizationStatusToString
{
    {OptimizationStatus::SUCCESS, "SUCCESS"s},
    {OptimizationStatus::NOT_ENOUGH_MEASUREMENTS, "NOT_ENOUGH_MEASUREMENTS"s},
    {OptimizationStatus::NOT_USABLE, "NOT_USABLE"s}
};


struct OptimizationResult
{
    OptimizationStatus optimizationStatus;
    std::string solutionReport;
    double sigmaZero;

};


class OptimizationProblem
{
    public:
        
        using ReprojectionErrors = std::vector<ReprojectionError>;
        using ObjectPointErrors = std::vector<ObjectPointError>;

    
        explicit OptimizationProblem(
            const ImagePointContainer& imagePointContainer,
            const CameraParametersContainer& cameraParametersContainer,
            ObjectPointContainer& objectPointContainer,
            Pose& pose);

        OptimizationResult solve();
        ReprojectionErrors computeReprojectionErrors() const;
        ObjectPointErrors computeObjectPointErrors() const;
        PoseCovariance computePoseCovariance();

    private:
        const ImagePointContainer& m_imagePointContainer;
        const CameraParametersContainer& m_cameraParametersContainer;
        ObjectPointContainer& m_objectPointContainer;
        
        Pose& m_poseRigInWorld; 
        Pose m_poseWorldInRig; //This is the pose that gonna to be optimized by ceres

        const ObjectPointContainer m_objectPointContainerOriginal;
        std::unordered_set<PointId> m_idsOfPointsUsedInOptimization;


        CameraParametersContainer m_cameraParametersContainerInvertedPoses;
        ceres::Problem m_problem;
        void buildProblem();
        Pose invertPose(const Pose& pose);
        void testReprojectionCostFunction(); //only for debug
        double m_sigmaZero{1.0};



};

}

//c : camera
//w : world
//r: rig

//T_w_c = T_w_r*T_r_c
//T_c_w = T_c_r*T_r_w