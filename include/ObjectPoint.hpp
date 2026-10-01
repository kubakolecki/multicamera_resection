#pragma once

#include "DataTypes.hpp"

#include <Eigen/Core>

#include <iomanip>

namespace multicamera_resection
{

struct ObjectPoint
{
    PointId id;
    Eigen::Vector3d point;
    Eigen::Vector3d uncertainty;

    void sentToStream(std::ostream &outputStream) const
    {
        std::string dlm{","};
        outputStream << id << dlm;
        outputStream << std::fixed << std::setprecision(5) << point(0) << dlm << point(1) << dlm << point(2) << dlm;
        outputStream << std::fixed << std::setprecision(5) << uncertainty(0) << dlm << uncertainty(1) << dlm << uncertainty(2) <<"\n";
    }
};



}