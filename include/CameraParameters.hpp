#pragma once

#include "DataTypes.hpp"

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <string>
#include <sstream>
#include <iomanip>

namespace multicamera_resection
{


struct CameraParameters
{
    CameraId id;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraMatrix{{1000.0,0.0,500.0},{0.0,1000.0,500.0},{0.0,0.0,1.0}};
    int imageWidth{1000};
    int imageHeight{1000};
    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;

    void sentToStream(std::ostream &outputStream) const
    {
        std::string dlm{","};
        outputStream << id <<dlm;
        outputStream << std::fixed <<std::setprecision(2) << cameraMatrix(0,0) <<dlm <<cameraMatrix(1,1) <<dlm<<cameraMatrix(0,2) <<dlm <<cameraMatrix(1,2) <<dlm;
        outputStream << imageWidth << dlm << imageHeight <<dlm;
        outputStream << std::setprecision(4);
        outputStream << position(0) <<dlm << position(1) <<dlm << position(2) <<dlm;
        outputStream << std::setprecision(13);
        outputStream << quaternion.w() << dlm <<quaternion.x() <<dlm <<quaternion.y() <<dlm <<quaternion.z() <<"\n";
    }
};



}