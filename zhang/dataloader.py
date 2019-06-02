import glob
import numpy as np
import cv2
import os

def dataloader(root='/Users/scenery/Code/SomeGit/Stereo/Problems/stereo/'):
    # set root path
    os.makedirs(root + "corner", exist_ok=True)
    root_origin = root + 'left/'
    root_corner = root + 'corner/'
    
    # root_undistort = '/Users/scenery/Code/SomeGit/Stereo/Problems/stereo/undistort/'

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = sorted(glob.glob(root_origin + '*.jpg'))

    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
            #cv2.imshow('img',img)
            #cv2.waitKey(500)
            cv2.imwrite(root_corner + 'image%d.jpg' % i, img)
        
        if ret == False:
            print('%d image does not contain enough feature points' % i)

    return objpoints, imgpoints