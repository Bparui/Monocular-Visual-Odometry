# 2D - 2D correspondences with feature tracking

## Algorithm
0. Set the 0^th^ frame as plotting reference as well 
current tracking-reference frame.
2. Set the 0^th^ frame as the tracking-reference frame.
3. Find keypoints for the reference frame  using **Fast algorithm**
4. Track the features in the reference frame into the k^th^ frame using **Optical flow** using **KLT**
5. Compute of Essential matrix using given Calibration Matrix and features.
6. Decompe of Essential Matrix into relative Rotation and Translation of k^th^ frame wrt current tracking-reference frame frame.
7. Update Rotation and translation to get pose of k^th^ wrt plotting reference frame i.e 0^th^ frame.
8. Repeat the above steps from  4-7 for 5\k successive frames.
9. On reaching the k+6^th^ frame implement steps 2,3 and update the tracking reference frame to the k+5^th frame . Repeat steps 4-8 again . 

## Results
1. KITTI00
![](https://i.imgur.com/cJazgV7.png)

2. KITTI05
![](https://i.imgur.com/ngGhfn3.png)

3. Raw camera data

![](https://i.imgur.com/lNtUrUC.png)
