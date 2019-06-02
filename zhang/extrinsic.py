import numpy as np 
import math

def get_extrinsic(H, cameraMatrix):
    # prepare the matrix and vector for calculation
    A_1 = np.linalg.inv(cameraMatrix)
    h_0 = H[:, 0]
    h_1 = H[:, 1]
    h_2 = H[:, 2]

    # calculate the extrinsic parameters
    # lambda_0 = 1 / np.linalg.norm(np.dot(A_1, h_0))
    lambda_0 = 1 / np.linalg.norm(np.dot(A_1, h_1))
    r_0 = lambda_0 * np.dot(A_1, h_0)
    r_1 = lambda_0 * np.dot(A_1, h_1)
    t = lambda_0 * np.dot(A_1, h_2)
    # r_2 = np.outer(r_0, r_1)
    r_2 = np.array([r_0[1] * r_1[2] - r_1[1] * r_0[2],
                    r_1[0] * r_0[2] - r_0[0] * r_1[2],
                    r_0[0] * r_1[1] - r_1[0] * r_0[1]
                    ])

    R = np.array([[r_0[0], r_0[1], r_0[2]],
                  [r_1[0], r_1[1], r_1[2]],
                  [r_2[0], r_2[1], r_2[2]]
                 ])

    return lambda_0, R, t