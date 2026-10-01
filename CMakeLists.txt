cmake_minimum_required(VERSION 3.30)
project(multicamera_resection VERSION 1.0.0)

add_executable(multicamera_resection
main.cpp
io.cpp
CameraParametersContainer.cpp
ImagePointContainer.cpp
ObjectPointContainer.cpp
OptimizationProblem.cpp
PoseCovariance.cpp
)

target_compile_definitions(multicamera_resection PRIVATE
    APP_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    APP_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    APP_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

set_property(TARGET multicamera_resection PROPERTY CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.4 REQUIRED)
find_package(Ceres 2.1 REQUIRED)


target_link_libraries(multicamera_resection PRIVATE Eigen3::Eigen)
target_link_libraries(multicamera_resection PRIVATE Ceres::ceres)
target_include_directories(multicamera_resection PRIVATE include)