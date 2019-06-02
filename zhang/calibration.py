import numpy as np 
from dataloader import *
from homography import *
from intrinsic import *
import sys

# load the image data
objpoints, imgpoints = dataloader()
# delete the axis full of zero
for i in range(len(objpoints)):
    objpoints[i] = np.delete(objpoints[i],2,axis=1)

# calculate and save the H matrix
Hs = []
for i in range(len(objpoints)):
    H = get_H(objpoints[i], imgpoints[i])
    Hs.append(H)

# get the camera matrix
cameraMatrix = get_intrinsic(Hs)

# print information
sys.stdout.write(
    "[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]\n"
    % (cameraMatrix[0][0], cameraMatrix[0][1], cameraMatrix[0][2], 
       cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2], 
       cameraMatrix[2][0], cameraMatrix[2][1], cameraMatrix[2][2])
    )
