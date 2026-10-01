#pragma once

#include "DataTypes.hpp"

namespace multicamera_resection
{

struct ObjectPointError
{
    PointId id;
    double eX{0.0};
    double eY{0.0};
    double eZ{0.0};
};

}