# Distance-based-EKF
Distance-based EKF for fast state estimation, which can be used to Odometry, SLAM, and Localization.

The advantages of the distance-based EKF are:<br>
    1, using the distance (is a scalar) as the measurement of EKF.   
    2, the distance can be defined by various forms. Thus, robust loss functions are used to redefine the distance to improve the ability to overcome the outliers, which is a novel and entirely different approach from the existing robust filters.
  
The C++ code depends only on PCL (https://github.com/PointCloudLibrary).

@InProceedings{Rusu_ICRA2011_PCL,   
  author    = {Radu Bogdan Rusu and Steve Cousins},   
  title     = {{3D is here: Point Cloud Library (PCL)}},   
  booktitle = {{IEEE International Conference on Robotics and Automation (ICRA)}},   
  month     = {May 9-13},   
  year      = {2011},   
  address   = {Shanghai, China},   
  publisher = {IEEE}   
}   
