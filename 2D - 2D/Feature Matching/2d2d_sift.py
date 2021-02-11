import numpy as np 
import cv2 as cv
from matplotlib import pyplot as plt
import math
import os
import glob

path = glob.glob("C:\\Users\\DELL\\Desktop\\VisualOdometry\\Visual-Odometry\\KITTI_sample\\image_0\\*.png")
#appending images in a list
images=[]
for file in path:
	
	image=cv.imread(file,cv.IMREAD_GRAYSCALE)
	images.append(image)
print(len(images))

def rel_scale(old_cloud, new_cloud):
    if old_cloud.shape[1]==5:
        print("its this")
        return 1
    else:
        min_idx = min([new_cloud.shape[0],old_cloud.shape[0]])
        Xm = new_cloud[:min_idx,:]
        Xn = np.roll(Xm,shift = -4)
        Xm1 = old_cloud[:min_idx,:]
        Xn1 = np.roll(Xm1,shift = -4)
        s = (np.linalg.norm(Xm1 - Xn1,axis = -1))/(np.linalg.norm(Xm - Xn,axis = -1)+1e-8)
        if math.isnan(np.median(s))==True:
            print("mhere")
            return 1
        s = np.median(s)
        return s


def triangulate(R_est,t_est,kp1,kp2,cam_mat):
    kp1  = kp1.astype(float)
    kp2  = kp2.astype(float)
    P1 = np.dot(cam_mat,np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]]))
    P2 = np.dot(cam_mat,np.hstack((R_est,t_est)))
    kp_1 = kp1.T
    kp_2 = kp2.T
    cloud = cv.triangulatePoints(P1, P2, kp_1,kp_2)
    return cloud.T

# calibration matrix
cam_mat = np.array([[7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02],           
                [0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02], 
                [0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00]])

#defining reference pose and initialising 1st parameters
t_old,t_old1 =  np.array([0,0,0]),np.array([0,0,0]) 
R_old = np.array([[1,0,0],[0,1,0],[0,0,1]])
x,x1,z,z1=[0],[0],[0],[0]
#loc = "C:\\Users\\DELL\\Desktop\\VisualOdometry\\Visual-Odometry\\KITTI_sample\\images\\000000.png"
img2 = images[0]
sift = cv.SIFT_create()
fast= cv.FastFeatureDetector_create(threshold = 50, nonmaxSuppression = True)
kp2, desc2 = sift.detectAndCompute(img2,None)
new_cloud = np.ones((5,5))


for i in range(1,len(images)):
    #finding correspondences
    print(i) 
    img1,kp1,desc1 = img2,kp2,desc2
    #loc = "C:\\Users\\DELL\\Desktop\\VisualOdometry\\Visual-Odometry\\KITTI_sample\\images\\"+str(i).zfill(6)+".png"
    img2 = images[i]
    kp2, desc2 = sift.detectAndCompute(img2,None)
    bf = cv.BFMatcher()
    matches = bf.knnMatch(desc1,desc2,2)

    # Applying Lowe's test
    good = []
    pts1 = []
    pts2 = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)

    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    
    # Pose Recovery Fundam_mat--> Essential_Mat --> Decomposition to R,t
    F, mask = cv.findFundamentalMat(pts1,pts2,cv.FM_RANSAC,0.4,0.9,mask =None)
    E = (cam_mat.T)@F@cam_mat
    r = 0
    if np.linalg.matrix_rank(E)==3:
        print("rank 3")
    pts11 = pts1[mask.ravel()==1]
    pts22 = pts2[mask.ravel()==1]
    points, R_est, t_est, mask_pose = cv.recoverPose(E, pts11,pts22,cam_mat)

    #Cloud updation --> Triangulation --> Scale recovery
    old_cloud = new_cloud
    new_cloud = triangulate(R_est,t_est,pts1,pts2,cam_mat)
    s = rel_scale(old_cloud,new_cloud)

    # Trajectory recovert 
    t_new = t_old - (s*(R_old@t_est)).T
    t_new1 = t_old1 - (R_old@t_est).T
    R_new = R_old@R_est

    
    # Creating list of Camera translation points
    x.append(-t_new[0,0])             # With relative scale
    z.append(t_new[0,2])
    x1.append(-t_new1[0,0])           # Without relative scale
    z1.append(t_new1[0,2])
    t_old1 = t_new1
    t_old = t_new
    R_old = R_new


plt.plot(x,z,'g')                # WIth relative scale                     GREEN
#plt.plot(x1,z1,'y')              # Without rel scale i.e scale = -1        YELLOW

#Groung truth plotting
data = np.loadtxt("C:\\Users\\DELL\\Desktop\\VisualOdometry\\Visual-Odometry\\KITTI_sample\\01.txt", delimiter=" ")
x_g = []
y_g = []
for i in range(len(images)):
    gtc_temp = data[i,:]
    x_g.append(gtc_temp[3])
    y_g.append(gtc_temp[11])

plt.plot(x_g,y_g,'r')

plt.show()