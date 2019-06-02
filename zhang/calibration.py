import numpy as np 
from dataloader import *
from homography import *
from intrinsic import *
from extrinsic import *
import sys
import argparse

# argument
parser = argparse.ArgumentParser()
parser.add_argument("--imgpath", type=str, default='/Users/scenery/Code/SomeGit/Stereo/Problems/stereo/', help="path to the origin image")
opt = parser.parse_args()

# load the image data
objpoints, imgpoints = dataloader(root=opt.imgpath)

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

# print camera matrix information
sys.stdout.write(
    "the camera matrix is: \n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]\n"
    % (cameraMatrix[0][0], cameraMatrix[0][1], cameraMatrix[0][2], 
       cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2], 
       cameraMatrix[2][0], cameraMatrix[2][1], cameraMatrix[2][2])
    )

# calculate the extrinsic
# currently only test the first view to judge the correctness
lambda_0, R, T = get_extrinsic(Hs[0], cameraMatrix)

# print extrinsic information
sys.stdout.write(
    "the rotation matrix is: \n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]\n"
    % (R[0][0], R[0][1], R[0][2], 
       R[1][0], R[1][1], R[1][2], 
       R[2][0], R[2][1], R[2][2])
    )
sys.stdout.write(
    "the lambda is: %f\n" % lambda_0)
sys.stdout.write(
    "the translation vector is: \n[%f, %f, %f]\n"
    % (T[0], T[1], T[2])
    )