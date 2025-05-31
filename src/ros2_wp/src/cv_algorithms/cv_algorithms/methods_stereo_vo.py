import os
import cv2
import time
import numpy as np
from datetime import datetime

def compute_disparity_map(img_left,img_right,matcher_name="bm", verbose=False):

    sadwindow= 6
    blockSize=11
    numDisparities = sadwindow*16

    if matcher_name == "bm":
        matcher= cv2.StereoBM.create(numDisparities= numDisparities,
                                    blockSize=blockSize)
    elif matcher_name=="sgbm":
        matcher= cv2.StereoSGBM.create(minDisparity=0,
                                      numDisparities=numDisparities,
                                      blockSize=blockSize,
                                      P1=8*blockSize**2,
                                      P2=32*blockSize**2,
                                      mode= cv2.StereoSGBM_MODE_SGBM_3WAY)
    #compute disparity map
    # CHANGE
    new_img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    new_img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
    # CHANGE
    if verbose:
        disparity_map= matcher.compute(new_img_left,new_img_right).astype(np.float32)/16
    else:
        disparity_map= matcher.compute(new_img_left,new_img_right).astype(np.float32)/16
        
    return disparity_map

def decomposeProjectionMatrix(P):
    k, r, t, _, _, _,_= cv2.decomposeProjectionMatrix(P)
    t=(t/t[3])[:3]
    return k,r,t

def compute_depth_map(disparity, k_left, t_left, t_right, rectified= True):
    if rectified:
        b= t_right[0].round(4)-t_left[2]
    else:
        b= t_left[0]-t_right[2].round(4)
    f= k_left[0][0]
    disparity[disparity==0]=0.1  # disparity 0, very far objects or matching errors
    disparity[disparity==-1]=0.1 #no valid correspondences found
    
    depth_map= np.zeros(disparity.shape)
    depth_map= b*f/disparity

    return depth_map

def stereo_to_depth(img_left,img_right,P0,P1,matcher="bm", verbose=False, rectification=True):
    #compute the disparity map
    disparity= compute_disparity_map(img_left,img_right,verbose=True)

    #compute k,r,t
    k_left,r_left,t_left= decomposeProjectionMatrix(P0)
    k_right,r_right,t_right= decomposeProjectionMatrix(P1)

    #compute the depth map
    depth= compute_depth_map(disparity, k_left, t_left, t_right, rectification)
    
    return depth
    
def extract_features(img1,detector_name= "orb", verbose=False):
    
    if detector_name== "orb":
        detector= cv2.ORB.create()
    elif detector_name=="sift":
        detector= cv2.SIFT.create()
        
    kp,des= detector.detectAndCompute(img1, mask=None)

    return kp, des

def match_features(des1,des2, matcher="BF", detector="orb", verbose=False):
    if matcher== "BF":
        if detector == "orb":
            bf = cv2.BFMatcher.create(cv2.NORM_HAMMING, crossCheck=False)
        elif detector== "sift":
            bf = cv2.BFMatcher.create(cv2.NORM_L2, crossCheck=False)
    matches= bf.knnMatch(des1,des2, k=2)

    return matches

#ratio test
def filter_matches(matches,threshold):
    filter_matches=[]
    for x, y in matches:
        if x.distance <= y.distance*threshold:
            filter_matches.append(x)
    return filter_matches

def estimate_motion(kp1,kp2,k,matches,depth, max_depth=3000):
    #compute the image in pixel
    image_points1= [kp1[m.queryIdx].pt for m in matches]
    image_points2= [kp2[m.trainIdx].pt for m in matches]

    # extracting parameters from k
    fx=k[0][0]
    fy=k[1][1]
    cx=k[0][2]
    cy=k[1][2]

    #converting interesting image_points1 from 2d to 3d
    objects_list=np.zeros((0,3))
    delete=[]

    for i , (u,v) in enumerate(image_points1):
        z= depth[int(round(v))][int(round(u))]

        if z >= max_depth:
            delete.append(i)
            continue

        x= z*(u-cx)/fx
        y= z*(v-cy)/fy
        objects_list= np.vstack([objects_list, np.array([x,y,z])])

    image_points1= np.delete(image_points1, delete, 0)
    image_points2= np.delete(image_points2, delete, 0)

    _, rvec,tvec, inliers= cv2.solvePnPRansac(objects_list, image_points2, k, None)
    rvec = cv2.Rodrigues(rvec)[0]

    return rvec, tvec


