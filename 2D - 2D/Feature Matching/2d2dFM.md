# 2D - 2D correspondences with feature matching

## Algorithm

0. Set the 0^th^ frame as the reference frame.
    #### **Then for every pair of frame starting from 0 to n^th**
1. Find keypoints and descriptors for k^th^ and k+1^th^ image using **Sift** feature detector.
2. Find Feature matches between k^th^ and k+1^th^ images using **Brute Force** (BF) matcher .
3. Application of  **RANSAC** and **Lowe's Test** for keypoint outlayer rejection.
4. Computation of Essential matrix using given Calibration Matrix and features.
5. Decomposition of Essential Matrix into relative Rotation and Translation of k_1^th^ frame wrt k^th^ frame.
6. Relative Scale Calculation using Triangulation + Distance Ratio
7. Rotation and translation updation to get pose wrt reference frame.


## Results

![](https://i.imgur.com/mI8q0I4.png,'KITTI00')

![](https://i.imgur.com/gwUVimu.png,'KITTI05')
