#include "PoseCovariance.hpp"

using namespace multicamera_resection;

Eigen::Vector3d PoseCovariance::getPositionStandardDeviations() const
{
    return Eigen::Vector3d{matrix(0,0), matrix(1,1), matrix(2,2)}.cwiseSqrt();
}

Eigen::Vector3d PoseCovariance::getRotationStandardDeviationsInRadians() const
{
    return Eigen::Vector3d{matrix(3,3), matrix(4,4), matrix(5,5)}.cwiseSqrt();
}