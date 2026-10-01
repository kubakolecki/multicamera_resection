## About Multicamera Resection
This app computes multicamera resection. Multicamera resection is a computer vision problem, sometimes referred to as PnP. In classical PnP the problem is to solve for camera pose with given 3D coordinates of object points
and the 2D image coordinates of their measurements. Classical PnP solution is provided for example by OpenCV. This app solves multicamera resection. It estimates the pose of a multicamera system. The number
of cameras is unlimited. By a multicamera system we mean not only a physical device that has two or more cameras. Multicamera system can also mean for example the trajectory estimated by a visual SLAM algorithm, like
ORB-SLAM3, where each camera has estimated pose. We may want to geo-reference this trajectory using resection and available control points (object points). Multicamera resection solves
for the pose of a multicamera system using 3D-2D correspondences. The aim of the algorithm is to find such pose of the system that the reprojection errors (i.e. errors in the 2D space) are minimized in the
least-square sense. The input data for the algorithm are:  
- coordinates of 3D points  
- coordinates of 2D points (image points)  
- relative poses of each camera in the system of cameras - for the multicamera device those are derived from the extrinsic calibration  
- calibration data of each camera  
- initial solution, which is needed because the problem we solve is not linear
  
There several important limitations of this app:
- it handles only pinhole camera model
- it doesn't handle distortion so provided image points must be undistorted
- it doesn't handle finding the initial pose so you must provide the initial pose

## Dependencies

I've tested building multicamera_resection in Linux. We used GCC compiler that is compatible with C++23. Use standard CMake build to compile the code. There are 2 external dependencies: Ceres Solver (for solving 
the optimization problem) and Eigen. You need to have Ceres Solver built with Suit Sparse because we use `ceres::SUITE_SPARSE` linear algebra type and `ceres::SPARSE_NORMAL_CHOLESKY` solver. If you don't want to
use Suit Sparse you can change code lines referring to setting Ceres Solver options and use `EIGEN_SPARSE` and dense QR solver, for example. The code does not depend on OpenCV.

## Running
After building you can use this app using following command line interface:  
`-c` : camera data file  
`-l` : object point (landmark) data file  
`-i` : image point file  
`-p` : file with initial pose data  
`-o` : output file 

You can use provided exemplary data. Data in the provided example represent camera trajectory recorded with stereo camera device. The structure of the files should be clear. Pose
is always represented as position + quaternion. File with image points have following column ordering: `camera_id, point_id, x, y`. Object points need to have uncertainties provided.
We use OpenCV image coordinate system convention. In the camera data file we use `fx, fy` to denote principal distances and `cx, cy` to denote coordinates of principal point.
Running the app from the terminal:

```
YOUR_BUILD_LOCATION/multicamera_resection -c SOURCE_CODE_DIR_LOCATION/multicamera_resection/exemplary_data/image_parameters_all_cameras.txt -l SOURCE_CODE_DIR_LOCATION/multicamera_resection/exemplary_data/reference_points_for_resection.txt -i SOURCE_CODE_DIR_LOCATION/multicamera_resection/exemplary_data/image_points.txt -p SOURCE_CODE_DIR_LOCATION/multicamera_resection/exemplary_data/initial_pose.txt -o ./result.txt
```

In the result file you will find reprojection residuals computed based on the initial pose, optimized pose, sigma zero parameter, Ceres Solver report, reprojection residuals after optimization.
