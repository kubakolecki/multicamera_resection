#pragma once

#include <Eigen/Geometry>

namespace multicamera_resection
{

struct Pose
{
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d position;
};

}