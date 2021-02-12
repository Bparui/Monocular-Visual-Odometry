# Visual Odometry

## What is Visual Odometry?
In robotics and computer vision, visual odometry is the process of determining the position and orientation  i.e pose of a robot/camera by analyzing the associated camera images.

## Types of VO based on Camera 
1. Monocular Visual Odometry 
2. Stereo Visual Odometry

This project is based on the implementation of the general pipeline of monocular visual odometry.
Monocular Visual Odometry
* A single camera = angle sensor 
* Motion scale is unobservable , must be synthesized 

## Basic Pipeline of Monocular Visual Odometry
1. Feature detection in images
2. Feature Matching / Feature tracking
3. Motion Estimation
    a. 2d - 2d Correspondences
    b. 2d - 3d Correspondences
    c. 3d - 3d Correspondences
4. Scale Correction (If required)
5. Bundle Adjustment



## Algorithms Implemented 
1. [2D-2D Correspondences using Feature Matching](https://github.com/Bparui/Monocular-Visual-Odometry/blob/main/2D%20-%202D/Feature%20Matching/2d2d_sift.py)
3. [2D-2D Correspondences using Feature Tracking](https://github.com/Bparui/Monocular-Visual-Odometry/blob/main/2D%20-%202D/Feature%20Tracking/V0_optflow_2-2.py) 
4. 3D-2D Correspondences using Feature Tracking

Primarily The KITTI Dataset has been used in this project . Along with the images the Ground truth has also been provided which has been used to verify the correctness of the plot. The dataset also Provides the Calibration Matrix which has directly been used.


---


Dataset: [KITTI](\link)














 



