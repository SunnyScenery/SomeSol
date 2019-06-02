import cv2 
import numpy as np 
import math

# add homogeneous / delete it
def hom(x):
    x = np.c_[x, np.ones(x.shape[0])]
    return x

def dehom(m):
    m = np.apply_along_axis(lambda x:x/x[3], 1, m)
    m = np.delete(m,3,axis=1)
    return m

# normalization function, not necessary for raw method?
def normalize(m):
    x = m[:, 0]
    y = m[:, 1]
    x_new = x.mean()
    y_new = y.mean()
    # (x):use method #2 to calculate the s_x and s_y value, because x, y might be different here 
    s_x = math.sqrt(2 / (x.var()) ** 2)
    s_y = math.sqrt(2 / (y.var()) ** 2)
    N_x = np.array([[s_x, 0. , -s_x * x_new],
                    [0. , s_y, -s_y * y_new],
                    [0. , 0. , 1.          ]])
    return N_x

def get_H(objpoints, imgpoints):
    # imgpoints may need to reshape
    imgpoints = imgpoints.reshape(42, 2)

    # objpoints and imgpoints are of a shape (42, 2)
    N_points = objpoints.shape[0]

    N_x = normalize(objpoints)
    N_u = normalize(imgpoints)

    # adding homogeneous value, after normalization to keep it to be 1, 
    # (x): or needing to calculate it back to 1
    objpoints = hom(objpoints)
    imgpoints = hom(imgpoints)

    # learn from another implementation by combine two transpose, it is clear
    objpoints = np.dot(objpoints, N_x)
    imgpoints = np.dot(imgpoints, N_u)

    # append method is low-efficient for numpy, change method
    M = np.zeros((N_points * 2, 9))
    for i in range(objpoints.shape[0]):
        X, Y, u, v = objpoints[i][0], objpoints[i][1], imgpoints[i][0], imgpoints[i][1]
        M[2 * i]     = np.array([-X, -Y, -1., 0., 0., 0., u * X, u * Y, u])
        M[2 * i + 1] = np.array([0., 0., 0., -X, -Y, -1., v * X, v * Y, v])

    # svd part
    u, s, vh = np.linalg.svd(M)
    k = np.argmin(s)
    H = vh[k]

    # denormalization
    H = H.reshape(3,3)
    H = np.dot(np.dot(np.linalg.inv(N_u), H), N_x)

    return H





