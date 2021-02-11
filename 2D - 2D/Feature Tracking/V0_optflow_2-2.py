import numpy as np 
import cv2 as cv
from matplotlib import pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import math
import glob

path = glob.glob("C:\\Users\\DELL\\Desktop\\VOdom\\KITTI_sample\\images\\*.png")
#appending images in a list
images=[]
for file in path:
	image=cv.imread(file,cv.IMREAD_GRAYSCALE)
	images.append(image)
print(len(images))
'''cap = cv.VideoCapture("C:\\Users\\DELL\\Desktop\\VisualOdometry\\Iv Dataset\\L_shaped_path.MP4")
images=[]
ret = True
i = 1
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    print(ret)
    # Our operations on the frame come here
    if ret:
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        print(gray.shape)
        images.append(gray)
    else:
        break

print(len(images))


# When everything done, release the capture
cap.release()
cv.destroyAllWindows()'''

def rel_scale(old_cloud, new_cloud):
    if old_cloud.shape[1]==5:
        return 1
    else:
        min_idx = min([new_cloud.shape[0],old_cloud.shape[0]])
        Xm = new_cloud[:min_idx,:]
        Xn = np.roll(Xm,shift = -3)
        Xm1 = old_cloud[:min_idx,:]
        Xn1 = np.roll(Xm1,shift = -3)
        s = (np.linalg.norm(Xm1 - Xn1,axis = -1))/(np.linalg.norm(Xm - Xn,axis = -1)+1e-8)
        if math.isnan(np.median(s))==True:
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

def find_pose(kp1,kp2,cam_mat,old_cloud,t_old,R_old,flag = 0):
    F, mask = cv.findFundamentalMat(kp1,kp2,cv.FM_RANSAC,0.4,0.9,mask =None)
    E = (cam_mat.T)@F@cam_mat
    r = 0
    if np.linalg.matrix_rank(E)==3:
        print("rank 3")
    kp1 = kp1[mask.ravel()==1]
    kp2 = kp2[mask.ravel()==1]

    points, R_est, t_est, mask_pose = cv.recoverPose(E, kp1,kp2,cam_mat)
    new_cloud = triangulate(R_est,t_est,kp1,kp2,cam_mat)
    if flag == 1:
        t_new = -(np.dot(R_old,t_est)).T
        R_new =R_est
    else:
        s = - rel_scale(old_cloud,new_cloud)
        t_new = t_old -(np.dot(R_old,t_est)).T
        R_new = R_old@R_est
    return t_new,R_new,new_cloud

def tracked_keypoint(img1,img2,kp1):
    #retval,pyr1 = cv.buildOpticalFlowPyramid(img1,winSize = (21,21),maxLevel = 3,withDerivatives = True)
    #retval2,pyr2 = cv.buildOpticalFlowPyramid(img2,winSize = (21,21),maxLevel = 3,withDerivatives = True)
    kp2,st,_ = cv.calcOpticalFlowPyrLK(img1, img2, np.float32(kp1), None, **lk_params)
    kp2 = kp2[st.ravel()==1]
    kp1 = kp1[st.ravel()==1]
    return kp2,kp1


cam_mat = np.array([[7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02],           
                [0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02], 
                [0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00]])

'''cam_mat = np.array([[518.56666108, 0., 329.45801792],[0., 518.80466479, 237.05589955],[  0., 0., 1.]])'''


feature_params = dict( maxCorners = 400,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
# Finding correspondences using Brute force with Knn (k = 2)
t_old =  np.array([0,0,0]) # list for all camera poses
R_old = np.array([[1,0,0],[0,1,0],[0,0,1]])
# 21/15,20,0.01    
lk_params = dict(winSize=(15,15), criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
x=[0]
z = [0]
#loc1 = "C:\\Users\\DELL\\Desktop\\VisualOdometry\\Visual-Odometry\\KITTI_sample\\images\\000000.png"
img1 = images[0]
fast= cv.FastFeatureDetector_create(threshold = 20, nonmaxSuppression = True)
#sift = cv.SIFT_create()
#kp1 = cv.goodFeaturesToTrack(img1, mask = None,maxCorners = 500,qualityLevel = 0.3,minDistance = 7,blockSize = 3,useHarrisDetector = True,k = 0.04)
kp1 = fast.detect(img1,None)
#loc2 = "C:\\Users\\DELL\\Desktop\\VisualOdometry\\Visual-Odometry\\KITTI_sample\\images\\000001.png"
img2 = images[1]
kp1 = np.array([kp1[idx].pt for idx in range(0, len(kp1))])
print(kp1,kp1.shape)
kp2,kp1 = tracked_keypoint(img1,img2,kp1)

old_cloud = np.zeros([1,1])
t_new,R_new,new_cloud = find_pose(kp1,kp2,cam_mat,old_cloud ,t_old,R_old,flag = 1)
x.append(-t_new[0,0])
z.append(t_new[0,2])
t_old = t_new
print(t_old)
R_old = R_new

for i in range(2,len(images)-1):
    #loc = "C:\\Users\\DELL\\Desktop\\VisualOdometry\\Visual-Odometry\\KITTI_sample\\images" + "\\"+ str(i).zfill(6)+".png"
    img1 = img2
    img2 = images[i]
    if i%5==0:
        #loc = "C:\\Users\\DELL\\Desktop\\VisualOdometry\\Visual-Odometry\\KITTI_sample\\images" + "\\"+ str(i-1).zfill(6)+".png"
        img1 = images[i-1]
        #kp1 = cv.goodFeaturesToTrack(img1, mask = None, **feature_params)
        kp1 = fast.detect(img1)
        kp1 = np.array([kp1[idx].pt for idx in range(0, len(kp1))])
    
    kp2,kp1 = tracked_keypoint(img1,img2,kp1)
    old_cloud = new_cloud
   
    t_new , R_new,new_cloud = find_pose(kp1,kp2,cam_mat,old_cloud,t_old,R_old)
    x.append(-t_new[0,0])
    z.append(t_new[0,2])
    t_old = t_new
    R_old = R_new

plt.plot(x,z,'g',label = 'With Opticalflow')

data = np.loadtxt("C:\\Users\\DELL\\Desktop\\VOdom\\KITTI_sample\\poses.txt", delimiter=" ")
x_g = []
y_g = []
for i in range(len(images)-1):
    gtc_temp = data[i,:]
    x_g.append(gtc_temp[3])
    y_g.append(gtc_temp[11])

plt.plot(x_g,y_g,'r',label = 'Ground Truth')
plt.legend()
plt.show()


