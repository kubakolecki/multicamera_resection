#include "io.hpp"

#include <fstream>
#include <ranges>
#include <iostream>
#include <iomanip>

using namespace multicamera_resection;

multicamera_resection::Pose multicamera_resection::readFromFile(const std::filesystem::path& pathToFile)
{
    static constexpr size_t numberOfPoseEntries{7uz};
    
    if (!std::filesystem::exists(pathToFile))
    {
        throw std::invalid_argument("Fatal Error. File with pose data does not exist!");
    }

    auto file{std::ifstream{pathToFile}};
    std::string header;
    std::string line;
    std::getline(file, header, '\n');
    std::getline(file, line, '\n');

    std::istringstream dataRecord;
    dataRecord.str(line);
    std::vector<std::string> entries;
    entries.reserve(numberOfPoseEntries);
    for (std::string entry; std::getline(dataRecord, entry, ',');)
    {
        entries.push_back(entry);
    }

    if (entries.size() != numberOfPoseEntries)
    {
        throw std::invalid_argument("Fatal error. Invalid number of entries for pose data!");
    }

    Pose pose{};
    for (const auto [index, entry] : entries | std::views::enumerate )
    {
        auto value{0.0};
        if (std::from_chars(entry.c_str(), entry.c_str() + entry.length(), value).ec == std::errc()) [[likely]]
        {
            if (std::isnan(value) || std::isinf(value))
            {
                throw std::invalid_argument("Not a number (nan) or inifinite (inf) value detected in pose data");
            }
            
            if (index < 3)
            {
                pose.position(index) = value;
            }
            else
            {
                if (value < -1.0 || value > 1.0)
                {
                    throw std::invalid_argument("Quaternion entries should be in the range from -1 to 1.");
                }
                if (index == 3)
                {
                    pose.quaternion.w() = value;
                }
                if (index == 4)
                {
                    pose.quaternion.x() = value;
                }
                if (index == 5)
                {
                    pose.quaternion.y() = value;
                }
                if (index == 6)
                {
                    pose.quaternion.z() = value;
                }
            }
        }
        else
        {
            throw std::invalid_argument("Not a numeric entry found for pose data, or data record is missformated!");
        }

    }
    return pose;
}

void multicamera_resection::print(const Pose& pose)
{
    std::cout <<"pose data:\n";
    std::cout <<"position: " << pose.position(0) <<" "<< pose.position(1) << " "<< pose.position(2) <<"\n";
    std::cout <<"quaternion: " << pose.quaternion.w() <<" "<< pose.quaternion.x() << " "<< pose.quaternion.y() <<" " << pose.quaternion.z() <<"\n";
}

void multicamera_resection::sendToStream(std::ostream &outputStream, const OptimizationResult& optimizationResult)
{
    outputStream << "Optimization status: " << optimizationStatusToString.at(optimizationResult.optimizationStatus) << "\n";
    outputStream << "\nSigma zero: " << optimizationResult.sigmaZero << "\n";
    outputStream << "\nSolution report:\n" << optimizationResult.solutionReport;
}

void multicamera_resection::sendToStream(std::ostream &outputStream, const Pose& pose)
{
    outputStream << "position_x,position_y,position_z,quaternion_w,quaternion_x,quaternion_y,quaternion_z\n";
    outputStream << std::fixed << std::setprecision(6);
    outputStream << pose.position(0) <<","<< pose.position(1) << ","<< pose.position(2) << ",";
    outputStream << std::fixed << std::setprecision(15); 
    outputStream << pose.quaternion.w() <<","<< pose.quaternion.x() << ","<< pose.quaternion.y() <<"," << pose.quaternion.z() <<"\n";
}

void multicamera_resection::sendToStream(std::ostream &outputStream, const OptimizationProblem::ReprojectionErrors& reprojectionErrors)
{
    outputStream << "point_id,camera_id,residual_x,residual_y\n";
    for (const auto& reprojectionError : reprojectionErrors)
    {
        outputStream << reprojectionError.id << "," << reprojectionError.cameraId << ",";
        outputStream << std::fixed << std::setprecision(3) << reprojectionError.eX<< "," << reprojectionError.eY << "\n";
    }
}

void multicamera_resection::sendToStream(std::ostream &outputStream, const OptimizationProblem::ObjectPointErrors& objectPointErrors)
{
    outputStream << "point_id,error_x,error_y,error_z\n";
    for (const auto& objectPointError : objectPointErrors)
    {
        outputStream << objectPointError.id << ",";
        outputStream << std::fixed << std::setprecision(5) << objectPointError.eX<< "," << objectPointError.eY << "," << objectPointError.eZ << "\n";
    }
}

void multicamera_resection::sendToStream(std::ostream &outputStream, const PoseCovariance& poseCovariance)
{
    outputStream << "Pose covariance matrix (row major order):\n";
    outputStream << std::fixed << std::setprecision(15);
    for (int i = 0; i < poseCovariance.matrix.rows(); ++i)
    {
        for (int j = 0; j < poseCovariance.matrix.cols(); ++j)
        {
            outputStream << poseCovariance.matrix(i, j);
            if (j < poseCovariance.matrix.cols() - 1)
            {
                outputStream << ",";
            }
        }
        outputStream << "\n";
    }

    outputStream << "\nStandard deviations for position (x, y, z):\n";
    Eigen::Vector3d positionStdDevs = poseCovariance.getPositionStandardDeviations();
    outputStream << std::fixed << std::setprecision(6);
    outputStream << positionStdDevs(0) << "," << positionStdDevs(1) << "," << positionStdDevs(2) << "\n";

    outputStream << "\nStandard deviations for rotation in degrees:\n";
    Eigen::Vector3d rotationStdDevs = poseCovariance.getRotationStandardDeviationsInRadians() * 180.0 / M_PI;
    outputStream << std::fixed << std::setprecision(6);
    outputStream << rotationStdDevs(0) << "," << rotationStdDevs(1) << "," << rotationStdDevs(2) << "\n";
}