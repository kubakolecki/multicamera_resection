#pragma once

#include <Eigen/Core>

namespace multicamera_resection
{
   class PoseCovariance
   {
        public:
            Eigen::Matrix<double, 6, 6, Eigen::RowMajor> matrix{};
            Eigen::Vector3d getPositionStandardDeviations() const;
            Eigen::Vector3d getRotationStandardDeviationsInRadians() const;
   };
}