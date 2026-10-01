#pragma once

#include "DataTypes.hpp"

#include <iomanip>

namespace multicamera_resection
{

struct ImagePoint
{
    PointId id;
    CameraId cameraId;
    double x;
    double y;
    double uncertainty{0.5};

    void sentToStream(std::ostream &outputStream) const
    {
        std::string dlm{","};
        outputStream << id << dlm << cameraId << dlm;
        outputStream << std::fixed << std::setprecision(2) << x << dlm << y <<"\n";
    }

};


}