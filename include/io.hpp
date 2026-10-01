#pragma once

#include "Pose.hpp"
#include "OptimizationProblem.hpp"

#include <filesystem>


namespace multicamera_resection
{
    //reads pose data from textfile
    //file should contain header
    //second line should contain:
    //x,y,z,qw,qx,qy,qz
    
    Pose readFromFile(const std::filesystem::path& pathToFile);
    void print(const Pose& pose);

    void sendToStream(std::ostream &outputStream, const OptimizationResult& optimizationResult);
    void sendToStream(std::ostream &outputStream, const Pose& pose);
    void sendToStream(std::ostream &outputStream, const OptimizationProblem::ReprojectionErrors& reprojectionErrors);
    void sendToStream(std::ostream &outputStream, const OptimizationProblem::ObjectPointErrors& objectPointErrors);
    void sendToStream(std::ostream &outputStream, const PoseCovariance& poseCovariance);



}