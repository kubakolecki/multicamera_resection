#include "io.hpp"
#include "OptimizationProblem.hpp"

#include <print>
#include <iostream>
#include <fstream>

using namespace std::literals;

int main(int argc, char **argv)
{
    //std::print("running multicamera resection app");

    //-c : camera data
    //-l : object point data
    //-i : image point data
    //-p : pose file
    //-o : output file

    static constexpr int requiredNumberOfCommandLineArguments{2*5};

    if (argc != requiredNumberOfCommandLineArguments + 1)
    {
        std::print("use following command line arguments:\n");
        std::print("-c : camera data file\n");
        std::print("-l : object point (landmark) data file\n");
        std::print("-i : image point file\n");
        std::print("-p : file with initial pose data\n");
        std::print("-o : output file\n");

        throw std::runtime_error("Invalid number of input command line arguments!" );
    }

    try
    {
        std::filesystem::path pathFileCameraData;
        std::filesystem::path pathFileObjectPointData;
        std::filesystem::path pathFileImagePointData;
        std::filesystem::path pathFileInitialPose;
        std::filesystem::path pathFileOutput;

        for (auto argId{1}; argId<requiredNumberOfCommandLineArguments+1; ++argId)
        {
            const auto arg{std::string(argv[argId])};
            if (arg == "-c"s)
            {
                pathFileCameraData = std::filesystem::path{std::string(argv[argId+1])};
            }
            if (arg == "-l"s)
            {
                pathFileObjectPointData = std::filesystem::path{std::string(argv[argId+1])};
            }
            if (arg == "-i"s)
            {
                pathFileImagePointData = std::filesystem::path{std::string(argv[argId+1])};
            }
            if (arg == "-p"s)
            {
                pathFileInitialPose = std::filesystem::path{std::string(argv[argId+1])};
            }
            if (arg == "-o"s)
            {
                pathFileOutput = std::filesystem::path{std::string(argv[argId+1])};
            }
        }

        multicamera_resection::Pose pose{multicamera_resection::readFromFile(pathFileInitialPose)};
        //std::print("Initial pose data:\n");
        //multicamera_resection::print(pose);

        multicamera_resection::ObjectPointContainer objectPointContainer;
        multicamera_resection::CameraParametersContainer cameraParametersContainer;
        multicamera_resection::ImagePointContainer imagePointContainer;
        objectPointContainer.readDataFromFile(pathFileObjectPointData);
        cameraParametersContainer.readDataFromFile(pathFileCameraData);
        imagePointContainer.readDataFromFile(pathFileImagePointData);
        
        /*
        std::print("\nObject points:\n");
        objectPointContainer.sentToStream(std::cout);
        std::print("\nCamera data:\n");
        cameraParametersContainer.sentToStream(std::cout);
        std::print("\nImage points:\n");
        imagePointContainer.sentToStream(std::cout);
        */

        std::ofstream outputFile{pathFileOutput};
        outputFile << "Initial pose:\n";
        multicamera_resection::sendToStream(outputFile, pose);


    
        multicamera_resection::OptimizationProblem optimizationProblem{imagePointContainer, cameraParametersContainer, objectPointContainer, pose};
        
        const auto reprojectionErrorsBeforeOptimization{optimizationProblem.computeReprojectionErrors()};
        outputFile << "\nReprojection errors before optimization:\n";
        multicamera_resection::sendToStream(outputFile, reprojectionErrorsBeforeOptimization);
        
        const auto optimizationResult{optimizationProblem.solve()};
        const auto reprojectionErrorsAfterOptimization{optimizationProblem.computeReprojectionErrors()};
        const auto objectPointErrorsAfterOptimization{optimizationProblem.computeObjectPointErrors()};
        
        const auto poseCovariance{optimizationProblem.computePoseCovariance()};

        
        //std::print("\nPose covariance:\n");
        //std::cout << poseCovariance.matrix << "\n";
        //std::print("\nPosition standard deviations: \n");
        //std::cout << poseCovariance.getPositionStandardDeviations().transpose() << "\n";
        //std::print("\nRotation standard deviations in degrees: \n");
        //std::cout << poseCovariance.getRotationStandardDeviationsInRadians().transpose() * 180.0 / M_PI << "\n";
        

        outputFile << "\nOptimized pose:\n";
        multicamera_resection::sendToStream(outputFile, pose);        
        
        outputFile << "\n";
        multicamera_resection::sendToStream(outputFile, optimizationResult);

        outputFile <<"\n";
        multicamera_resection::sendToStream(outputFile, poseCovariance);

        outputFile << "\n\nReprojection errors after optimization:\n";
        multicamera_resection::sendToStream(outputFile, reprojectionErrorsAfterOptimization);

        outputFile << "\n\nObject point errors after optimization:\n";
        multicamera_resection::sendToStream(outputFile, objectPointErrorsAfterOptimization);



        outputFile.close();
        


    }
    catch(const std::exception& e)
    {
        const auto exceptionInfo{std::string{e.what()}};
        std::print("\nException: {}\n", exceptionInfo);
        std::print("Exiting with fatal error.\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
    
}
