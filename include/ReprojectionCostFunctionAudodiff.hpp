#pragma once

#include "CameraParameters.hpp"
#include "ImagePoint.hpp"

#include <Eigen/Geometry>

namespace multicamera_resection
{

class ReprojectionCostFunctionAutodiff
{
    public:
        explicit ReprojectionCostFunctionAutodiff(const CameraParameters& cameraParameters, const ImagePoint& imagePoint):
         m_cameraParameters{cameraParameters}, m_imagePoint{imagePoint}
         {}
         
        template<typename T>
	    bool operator() (const T* const position_r_rw, const T* const quaternion_r_w,  const T* const position_w_wp, T* residual) const
        {
            T array_x_r_rw[3]{position_r_rw[0], position_r_rw[1], position_r_rw[2]};
            T array_q_r_w[4]{quaternion_r_w[0], quaternion_r_w[1], quaternion_r_w[2], quaternion_r_w[3]};
            T array_x_w_wp[3]{position_w_wp[0], position_w_wp[1], position_w_wp[2]};

            //variables:
            Eigen::Map<Eigen::Matrix<T, 3, 1>> x_r_rw{array_x_r_rw};
            Eigen::Map<Eigen::Quaternion<T>> q_r_w{array_q_r_w};
            Eigen::Map<Eigen::Matrix<T, 3, 1>> x_w_wp{array_x_w_wp};

            //constants:
            const Eigen::Quaternion<T> q_c_r{m_cameraParameters.quaternion.template cast<T>()};
            const Eigen::Matrix<T, 3, 1> x_c_cr {m_cameraParameters.position.template cast<T>()};

            Eigen::Matrix<T, 3, 1> x_c_cp {x_c_cr + q_c_r*(q_r_w * x_w_wp) + q_c_r*x_r_rw};

            residual[0] = T(m_cameraParameters.cameraMatrix(0,2)) + T(m_cameraParameters.cameraMatrix(0,0))*(x_c_cp(0)/x_c_cp(2)) - T(m_imagePoint.x);
            residual[1] = T(m_cameraParameters.cameraMatrix(1,2)) + T(m_cameraParameters.cameraMatrix(1,1))*(x_c_cp(1)/x_c_cp(2)) - T(m_imagePoint.y);

            residual[0] /= T(m_imagePoint.uncertainty);
            residual[1] /= T(m_imagePoint.uncertainty);

            return true;
        }


    private:
        CameraParameters m_cameraParameters;
        ImagePoint m_imagePoint;


};


}