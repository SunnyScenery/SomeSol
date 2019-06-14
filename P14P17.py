import glob
import numpy as np
import cv2
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--imgpath", type=str, default='/Users/scenery/Code/SomeGit/Stereo/Problems/stereo/', help="path to the origin image")
opt = parser.parse_args()

root_left_origin = opt.imgpath + 'left/'
root_right_origin = opt.imgpath + 'right/'
# root_origin = '/Users/scenery/Downloads/left/'
# root_corner = '/Users/scenery/Downloads/corner/'
root_corner = opt.imgpath + 'corner/'
os.makedirs(root_corner, exist_ok=True)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints_left = [] # 2d points in image plane.
imgpoints_right = []

# images = sorted(glob.glob(root_origin + '*.jpg'))
images_left = sorted(glob.glob(root_left_origin + '*.jpg'))
images_right = sorted(glob.glob(root_right_origin + '*.jpg'))

for i, fname in enumerate(images_left):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        
        # left image ok, go check the right image
        img_r = cv2.imread(images_right[i])
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        ret_r, corners_r = cv2.findChessboardCorners(gray_r, (7,6),None)
        
        # both images pass
        if ret_r == True:
            corners2_r = cv2.cornerSubPix(gray_r, corners_r, (11,11), (-1,-1), criteria)
            imgpoints_right.append(corners2_r)

            # Draw and display the corners
            img_r = cv2.drawChessboardCorners(img_r, (7,6), corners2_r, ret_r)
            #cv2.imshow('img',img)
            #cv2.waitKey(500)
            cv2.imwrite(root_corner + 'right_image%d.jpg' % i, img_r)
        
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints_left.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
            #cv2.imshow('img',img)
            #cv2.waitKey(500)
            cv2.imwrite(root_corner + 'left_image%d.jpg' % i, img)
        
        if ret_r == False:
            print('%d right image is false' % i)
    
    if ret == False:
        print('%d left image is false' % i)
# in latest document: https://docs.opencv.org/4.1.0/d9/d0c/group__calib3d.html#ga246253dcc6de2e0376c599e7d692303a 
ret_l, mtx_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(objpoints, imgpoints_left, gray.shape[::-1],None,None)
ret_r, mtx_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(objpoints, imgpoints_right, gray.shape[::-1],None,None)

retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right, mtx_l, dist_l, mtx_r, dist_r, imageSize = gray.shape[::-1])

# ---
# rectification
# ---

R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, gray.shape[::-1], R, T)
rmap1, rmap2 = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, gray.shape[::-1], 5)
lmap1, lmap2 = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, gray.shape[::-1], 5)

for i, fname in enumerate(images_left):
    limg = cv2.imread(fname)
    left_dst = cv2.remap(limg, lmap1, lmap2, 1)
    cv2.imwrite(root_corner + 'rectified_left%d.png' % i,left_dst)

    rimg = cv2.imread(images_right[i])
    right_dst = cv2.remap(rimg, rmap1, rmap2, 1)
    cv2.imwrite(root_corner + 'rectified_right%d.png' % i,right_dst)

    # ---
    # SGBM
    # ---
    SGBM = cv2.StereoSGBM_create(minDisparity = 2,
                                numDisparities = 16,
                                blockSize = 5,
                                P1 = 1,
                                P2 = 2,
                                disp12MaxDiff = 10,
                                preFilterCap = 0,
                                uniquenessRatio = 0,
                                speckleWindowSize = 0,
                                speckleRange = 0
                                )
    disp_img = SGBM.compute(left_dst, right_dst)
    # disp_img = cv2.StereoSGBM.compute(left_dst, right_dst)
    cv2.imwrite(root_corner + 'disparityMap%d.png' % i, disp_img)
