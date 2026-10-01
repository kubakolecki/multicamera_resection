#pragma once

#include "DataTypes.hpp"

namespace multicamera_resection
{

struct ReprojectionError
{
    PointId id;
    CameraId cameraId;
    double eX{0.0};
    double eY{0.0};
};

}