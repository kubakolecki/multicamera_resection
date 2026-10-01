#pragma once

#include "ObjectPoint.hpp"

#include <ceres/ceres.h>

namespace multicamera_resection
{

class ObjectPointCostFunction: public ceres::SizedCostFunction<3,3>
{
    public:
        explicit ObjectPointCostFunction(const ObjectPoint& objectPoint) : m_objectPoint{objectPoint}{}

        virtual ~ObjectPointCostFunction() {}

        bool Evaluate (double const* const* coordiantes, double* residuals, double** jacobian) const override
        {
            
            residuals[0] = coordiantes[0][0] - m_objectPoint.point(0);
            residuals[1] = coordiantes[0][1] - m_objectPoint.point(1);
            residuals[2] = coordiantes[0][2] - m_objectPoint.point(2);
            
            residuals[0] /= m_objectPoint.uncertainty(0);
            residuals[1] /= m_objectPoint.uncertainty(1);
            residuals[2] /= m_objectPoint.uncertainty(2);


            if (jacobian != nullptr  && jacobian[0] != nullptr)
            {
                jacobian[0][0] = 1.0/m_objectPoint.uncertainty(0);
                jacobian[0][1] = 0.0;
                jacobian[0][2] = 0.0;

                jacobian[0][3+0] = 0.0;
                jacobian[0][3+1] = 1.0/m_objectPoint.uncertainty(1);
                jacobian[0][3+2] = 0.0;

                jacobian[0][6+0] = 0.0;
                jacobian[0][6+1] = 0.0;
                jacobian[0][6+2] = 1.0/m_objectPoint.uncertainty(2);
            }
                    
            return true;
        }


    private:
        ObjectPoint m_objectPoint;

};


}