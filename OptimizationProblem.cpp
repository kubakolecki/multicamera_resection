#include "OptimizationProblem.hpp"
#include "ReprojectionCostFunctionAudodiff.hpp"
#include "ObjectPointCostFunction.hpp"

#include <Eigen/Geometry> //for inversion of transformations
#include <ceres/covariance.h>

using namespace multicamera_resection;

OptimizationProblem::OptimizationProblem(
    const ImagePointContainer& imagePointContainer,
    const CameraParametersContainer& cameraParametersContainer,
    ObjectPointContainer& objectPointContainer,
    Pose& pose): m_imagePointContainer{imagePointContainer}, m_cameraParametersContainer{cameraParametersContainer},
    m_objectPointContainer{objectPointContainer}, m_poseRigInWorld{pose}, m_poseWorldInRig{invertPose(m_poseRigInWorld)}, m_objectPointContainerOriginal{objectPointContainer}
    { 
        for (const auto& imagePoint: m_imagePointContainer.getData())
        {
            if (!m_cameraParametersContainer.getData().contains(imagePoint.cameraId))
            {
                throw std::invalid_argument("Fatal error. Image point with id " + imagePoint.id + " references camera with id " + imagePoint.cameraId + " for which no camera parameters are provided!");
            }

            //Here we could also check if object point with provided id exists, but we can also just skip such points during construction of the problem.
            //User is allowed to provide image points with object point ids for which no object point data is provided. Such points will be just ignored during construction of the problem and will not contribute to the optimization.
            //This allows user to exclude some points from optimization by not providing object point data for them.

        }

        //In case of reprojection error it is more convenient to inverse the transformation before
        //constructing cost functors.
        //The cameraParametersContainerInvertedPoses will contain position of rig in cs of each camera
        //and rotation of rig in cs of each camera 
        m_cameraParametersContainerInvertedPoses = m_cameraParametersContainer;
        for (auto& [id, cameraParameters] : m_cameraParametersContainerInvertedPoses.getData())
        {   Pose poseCameraInRig{cameraParameters.quaternion, cameraParameters.position};
            const auto poseRigInCamera{invertPose(poseCameraInRig)};
            cameraParameters.position = poseRigInCamera.position;
            cameraParameters.quaternion = poseRigInCamera.quaternion;
        }

        //testReprojectionCostFunction(); //only for debug
    }


void OptimizationProblem::buildProblem()
{
    m_idsOfPointsUsedInOptimization.clear();
    
    for (const auto& imagePoint : m_imagePointContainer.getData())
    {
        if (!m_objectPointContainer.getData().contains(imagePoint.id))
        {
            continue;
        }
        
        m_idsOfPointsUsedInOptimization.insert(imagePoint.id);

        auto& objectPoint {m_objectPointContainer.getData().at(imagePoint.id)};
        auto& cameraParameters {m_cameraParametersContainerInvertedPoses.getData().at(imagePoint.cameraId)};

        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<ReprojectionCostFunctionAutodiff,2,3,4,3>(
            new ReprojectionCostFunctionAutodiff{cameraParameters, imagePoint}
        );

        ceres::LossFunction* lossFunction{nullptr};

        m_problem.AddResidualBlock(costFunction, lossFunction, m_poseWorldInRig.position.data(), m_poseWorldInRig.quaternion.coeffs().data(), objectPoint.point.data());
        m_problem.AddParameterBlock(m_poseWorldInRig.quaternion.coeffs().data(), 4, new ceres::EigenQuaternionManifold{});
    }

    
    for (auto& [id, objectPoint] : m_objectPointContainer.getData())
    {
        ceres::CostFunction* costFunction = new ObjectPointCostFunction{objectPoint};
        ceres::LossFunction* lossFunction{nullptr};
        m_problem.AddResidualBlock(costFunction, lossFunction, objectPoint.point.data());
    }
    
}

    
OptimizationResult OptimizationProblem::solve()
{
    if (m_imagePointContainer.getData().size() < 4)
    {
        return {OptimizationStatus::NOT_ENOUGH_MEASUREMENTS, std::string{}, 0.0};
    }

    ceres::Solver::Summary ceresSummary;
    ceres::Solver::Options ceresSolverOptions;

    ceresSolverOptions.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    ceresSolverOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceresSolverOptions.minimizer_progress_to_stdout = false;

    buildProblem();
    ceres::Solve(ceresSolverOptions, &m_problem, &ceresSummary);

    if (!ceresSummary.IsSolutionUsable())
    {
        return {OptimizationStatus::NOT_USABLE, ceresSummary.FullReport(), 0.0};
    }

    m_poseRigInWorld = invertPose(m_poseWorldInRig); //converting back optimized pose

    m_sigmaZero  = std::sqrt(2.0 * ceresSummary.final_cost / (ceresSummary.num_residuals - ceresSummary.num_effective_parameters));

    
    return {OptimizationStatus::SUCCESS, ceresSummary.FullReport(), m_sigmaZero};
}

Pose OptimizationProblem::invertPose(const Pose& pose)
{
    Eigen::Isometry3d transformation;
    transformation.linear() = pose.quaternion.toRotationMatrix();
    transformation.translation() = pose.position;
    const auto transformationInverted {transformation.inverse()};
    Eigen::Quaterniond q{transformationInverted.linear()};
    Eigen::Vector3d t{transformationInverted.translation()};
    return Pose{q,t};
    
}

OptimizationProblem::ReprojectionErrors OptimizationProblem::computeReprojectionErrors() const
{
    OptimizationProblem::ReprojectionErrors reprojectionErrors;
    reprojectionErrors.reserve(m_imagePointContainer.size());
    
    for (const auto& imagePoint : m_imagePointContainer.getData())
    {
        if (!m_objectPointContainer.getData().contains(imagePoint.id))
        {
            continue;
        }
        
        auto& objectPoint {m_objectPointContainer.getData().at(imagePoint.id)};
        auto& cameraParameters {m_cameraParametersContainerInvertedPoses.getData().at(imagePoint.cameraId)};
        auto costFunction {ReprojectionCostFunctionAutodiff{cameraParameters, imagePoint}};
        double residuals[2];
        costFunction(m_poseWorldInRig.position.data(), m_poseWorldInRig.quaternion.coeffs().data(), objectPoint.point.data(), residuals);
        residuals[0] *= imagePoint.uncertainty;
        residuals[1] *= imagePoint.uncertainty;
        reprojectionErrors.emplace_back(imagePoint.id, imagePoint.cameraId, residuals[0], residuals[1]);

    }

    std::sort(reprojectionErrors.begin(), reprojectionErrors.end(), [](const ReprojectionError& v1, const ReprojectionError& v2)
    {
        const double errorNorm1 {std::sqrt(v1.eX * v1.eX + v1.eY * v1.eY)};
        const double errorNorm2 {std::sqrt(v2.eX * v2.eX + v2.eY * v2.eY)};
        return errorNorm1 > errorNorm2;
    });

    return reprojectionErrors;
}

OptimizationProblem::ObjectPointErrors OptimizationProblem::computeObjectPointErrors() const
{
    OptimizationProblem::ObjectPointErrors objectPointErrors;
    objectPointErrors.reserve(m_objectPointContainer.getData().size());

    for (const auto& [id, objectPoint] : m_objectPointContainer.getData())
    {
        if (!m_idsOfPointsUsedInOptimization.contains(id))
        {
            continue;
        }
        
        const auto& objectPointOriginal {m_objectPointContainerOriginal.getData().at(id)};
        const double eX = objectPoint.point(0) - objectPointOriginal.point(0);
        const double eY = objectPoint.point(1) - objectPointOriginal.point(1);
        const double eZ = objectPoint.point(2) - objectPointOriginal.point(2);
        objectPointErrors.emplace_back(id, eX, eY, eZ);
    }

    std::sort(objectPointErrors.begin(), objectPointErrors.end(), [](const ObjectPointError& v1, const ObjectPointError& v2)
    {
        const double errorNorm1 {std::sqrt(v1.eX * v1.eX + v1.eY * v1.eY + v1.eZ * v1.eZ)};
        const double errorNorm2 {std::sqrt(v2.eX * v2.eX + v2.eY * v2.eY + v2.eZ * v2.eZ)};
        return errorNorm1 > errorNorm2;
    });

    return objectPointErrors;
}

PoseCovariance OptimizationProblem::computePoseCovariance()
{
    ceres::Covariance::Options covarianceOptions;
    covarianceOptions.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    covarianceOptions.algorithm_type = ceres::DENSE_SVD;

    ceres::Covariance covariance(covarianceOptions);

    std::vector<std::pair<const double*, const double*> > covariance_blocks;

    covariance_blocks.emplace_back(m_poseWorldInRig.position.data(), m_poseWorldInRig.position.data());
    covariance_blocks.emplace_back(m_poseWorldInRig.quaternion.coeffs().data(), m_poseWorldInRig.quaternion.coeffs().data());
    covariance_blocks.emplace_back(m_poseWorldInRig.position.data(), m_poseWorldInRig.quaternion.coeffs().data());

    if (!covariance.Compute(covariance_blocks, &m_problem))
    {
        throw std::runtime_error("Failed to compute covariance!");
    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> positionCovariance;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> quaternionCovariance;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> positionQuaternionCovariance;

    covariance.GetCovarianceBlock(m_poseWorldInRig.position.data(), m_poseWorldInRig.position.data(), positionCovariance.data());
    covariance.GetCovarianceBlockInTangentSpace(m_poseWorldInRig.quaternion.coeffs().data(), m_poseWorldInRig.quaternion.coeffs().data(), quaternionCovariance.data());
    covariance.GetCovarianceBlockInTangentSpace(m_poseWorldInRig.position.data(), m_poseWorldInRig.quaternion.coeffs().data(), positionQuaternionCovariance.data());

    PoseCovariance poseWorldInRigCovariance;

    poseWorldInRigCovariance.matrix.block<3,3>(0,0) = positionCovariance;
    poseWorldInRigCovariance.matrix.block<3,3>(3,3) = quaternionCovariance;
    poseWorldInRigCovariance.matrix.block<3,3>(0,3) = positionQuaternionCovariance;
    poseWorldInRigCovariance.matrix.block<3,3>(3,0) = positionQuaternionCovariance.transpose();

    poseWorldInRigCovariance.matrix *= (m_sigmaZero * m_sigmaZero);

    //we have computed covariance of the inverse of the pose,
    //to get the covarinace of pose we have to applay covarinace propagation
    //for that we need to compute adjoint of the inversion transformation at the solution

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotationMatrix = m_poseWorldInRig.quaternion.toRotationMatrix();

    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> adjoint;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> positionSkewSymmetric;
    positionSkewSymmetric << 0.0, -m_poseWorldInRig.position(2), m_poseWorldInRig.position(1),
    m_poseWorldInRig.position(2), 0.0, -m_poseWorldInRig.position(0),
    -m_poseWorldInRig.position(1), m_poseWorldInRig.position(0), 0.0;

    adjoint.block<3,3>(0,0) = rotationMatrix;
    adjoint.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
    adjoint.block<3,3>(0,3) = positionSkewSymmetric * rotationMatrix;
    adjoint.block<3,3>(3,3) = rotationMatrix;

    PoseCovariance poseRigInWorldCovariance;
    poseRigInWorldCovariance.matrix = adjoint * poseWorldInRigCovariance.matrix * adjoint.transpose();

    return poseRigInWorldCovariance;
}

void OptimizationProblem::testReprojectionCostFunction()//only for debug
{
    auto objectPoint {ObjectPoint{}};
    objectPoint.id = "1"s;
    objectPoint.point = Eigen::Vector3d{100.0,100.0,0.0};
    objectPoint.uncertainty = Eigen::Vector3d{0.02,0.02,0.02};

    auto cameraParameters {CameraParameters{}};
    cameraParameters.id = "1"s;
    cameraParameters.cameraMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>{{556.889410,0.0,628.188591},{0.0,556.889410,479.538639},{0.0,0.0,1.0}};
    cameraParameters.imageWidth = 1280;
    cameraParameters.imageHeight = 960;
    cameraParameters.position = Eigen::Vector3d{-0.5,-0.1,0.0};
    cameraParameters.quaternion.x() = 0.0;
    cameraParameters.quaternion.y() = 0.0;
    cameraParameters.quaternion.z() = 0.0;
    cameraParameters.quaternion.w() = 1.0;

    auto pose{Pose{}};
    pose.position = Eigen::Vector3d{100.0,100.0,-50.0};
    pose.quaternion.x() = 0.0866574946722045;
    pose.quaternion.y() = -0.0196611609090833;
    pose.quaternion.z() = 0.0245528092012491;
    pose.quaternion.w() = 0.9957414709296641;

    auto poseInv{invertPose(pose)};

    auto imagePoint{ImagePoint{}};
    imagePoint.id = "1"s;
    imagePoint.cameraId = "1"s;
    imagePoint.x = 647.0929384338067;
    imagePoint.y = 575.5090391596393;

    double residuals[2];

    auto costFunction {ReprojectionCostFunctionAutodiff{cameraParameters, imagePoint}};
    costFunction(poseInv.position.data(), poseInv.quaternion.coeffs().data(), objectPoint.point.data(), residuals);

    std::cout << "Testing reprojection cost function with known solution. Expected residuals should be close to zero." << "\n";
    std::cout << "residuals: " <<std::fixed <<std::setprecision(15) << residuals[0] << ", " << residuals[1] <<"\n" << std::endl;

}

