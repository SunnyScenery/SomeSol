import numpy as np
import math

def get_intrinsic(Hs):
    M_views = len(Hs)
    V = np.zeros((2 * M_views, 6))
    for i in range(M_views):
        H = Hs[i]
        V[2 * i] = np.array([H[0][0] * H[0][1], 
                             H[0][0] * H[1][1] + H[1][0] * H[0][1],
                             H[1][0] * H[1][1],
                             H[2][0] * H[0][1] + H[0][0] * H[2][1],
                             H[2][0] * H[1][1] + H[1][0] * H[2][1],
                             H[2][0] * H[2][1],
                             ])
        V[2 * i + 1] = (np.array([H[0][0] * H[0][0], 
                                 H[0][0] * H[1][0] + H[1][0] * H[0][0],
                                 H[1][0] * H[1][0],
                                 H[2][0] * H[0][0] + H[0][0] * H[2][0],
                                 H[2][0] * H[1][0] + H[1][0] * H[2][0],
                                 H[2][0] * H[2][0],
                                 ]) - 
                       np.array([H[0][1] * H[0][1], 
                                 H[0][1] * H[1][1] + H[1][1] * H[0][1],
                                 H[1][1] * H[1][1],
                                 H[2][1] * H[0][1] + H[0][1] * H[2][1],
                                 H[2][1] * H[1][1] + H[1][1] * H[2][1],
                                 H[2][1] * H[2][1],
                             ]))
    u, s, vh = np.linalg.svd(V)
    k = np.argmin(s)
    B = vh[k]

    w = B[0] * B[2] * B[5] - B[1] * B[1] * B[5] - B[0] * B[4] * B[4] + 2 * B[1] * B[3] * B[4] - B[2] * B[3] * B[3]
    d = B[0] * B[2] - B[1] * B[1]

    alpha = math.sqrt(w / (d * B[0]))
    beta = math.sqrt(w * B[0] / (d ** 2))
    yita = B[1] * math.sqrt(w / ((d ** 2) * B[0]))
    u_c = (B[1] * B[4] - B[2] * B[3]) / d
    v_c = (B[1] * B[3] - B[0] * B[4]) / d

    cameraM = np.array([[alpha, yita, u_c],
                        [0.   , beta, v_c],
                        [0.   , 0.  , 1. ]])

    return cameraM