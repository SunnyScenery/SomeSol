# Work Doc

## plan

2-3h everyday, more on weekends

w1d1-w1d5: finish the camera basic part

w1d6-w1d7: BB 9-12

w2d1-w2d5: BB13-16

w2d6-w2d7 and remained: stereo matching

## Sol

#### P1

the intrinsics is $$K = \left[ \begin{matrix}f_x&&p_x \\&f_y&p_y\\&&1\end{matrix} \right] \tag{intrinsics matrix}$$ 

where f is the focal length and p is for moving the camera origin to image origin

consider that pixel size could be different i.e. the pixels per meter, a new one is

$$\left[ \begin{matrix}m_x&&\\&m_y&\\&&1\end{matrix} \right] \left[ \begin{matrix}f&&p_x \\&f&p_y\\&&1\end{matrix} \right] = \left[ \begin{matrix}\alpha_x&&\beta_x \\&\alpha_y&\beta_y\\&&1\end{matrix} \right] $$ 

where m is the pixels per length

the extrinsics is 

$$\left[ \begin{matrix}R&T\\0&1\end{matrix} \right] \tag{extrinsics matrix}$$

where R is a rotation matrix and T is the translation matrix

finally, the camera matrix is 

$$ P = KR[I|T] \tag{translation first}$$

Where K is the intrinsics matrix, R is the rotation matrix, I is the identity matrix, T is the translation matrix

#### P2

In general,
$$
\alpha \left[ \begin{matrix} u\nonumber\\v\\1 \end{matrix} \right] = \left[ \begin{matrix}f_x&&c_x \\&f_y&c_y\\&&1\end{matrix} \right] \left[ \begin{matrix}R|t\end{matrix} \right] \left[ \begin{matrix}X_1\\X_2\\X_3\\1\end{matrix} \right]
$$
after transform the world coordinates into camera coordinates,
$$
\left[ \begin{matrix} X\\Y\\Z \end{matrix} \right] = \left[ \begin{matrix}R_{11}X_1+R_{12}X_2+R_{13}X_3+T_1\\R_{21}X_1+R_{22}X_2+R_{23}X_3+T_2\\R_{31}X_1+R_{32}X_2+R_{33}X_3+T_3\end{matrix} \right]
$$
(X,Y,Z) is the point in the camera coordinates.

By the similarity, $$\alpha$$ is actually Z. So we have,
$$
Z \left[ \begin{matrix} u\nonumber\\v\\1 \end{matrix} \right] = \left[ \begin{matrix}f_x&&c_x \\&f_y&c_y\\&&1\end{matrix} \right] \left[ \begin{matrix} X\nonumber\\Y\\Z \end{matrix} \right]
$$
and considering the different direction, 
$$
\left[ \begin{matrix} u\\v \end{matrix} \right] = -\left[ \begin{matrix}\frac{f_x}{Z}X+C_x\\\frac{f_y}{Z}Y+C_y\end{matrix} \right]
$$
By (1) and (2), we can get the (u,v) on the image plane

### P3

It is a ray from the camera through the point on the image plane to the infinity
$$
\left[ \begin{matrix} \frac{X}{Z}\\\frac{Y}{Z} \end{matrix} \right] = \left[ \begin{matrix}\frac{-u-C_x}{f_x}\\\frac{-v-C_y}{f_y}\end{matrix} \right]
$$
(X,Y,Z) is the point in the camera coordinates. X and Y will change with Z changing.

### P4

There are two common radial distortions, positive radial distortion and negative radial distortion. There are also other types of distortion, such as tangential distortion. It is because the camera is not perfectly parallel to the image plane so that  some areas will be closer in picture.

for radial distortion:
$$
x' = x(1+k_1r^2+k_2r^4+k_3r^6)\nonumber\\
y' = y(1+k_1r^2+k_2r^4+k_3r^6)\\
r^2 = x^2 + y^2
$$
for tangential distortion:
$$
x' = x + [2p_1xy + p_2(r^2+2x^2)]\nonumber\\
y' = y + [p_1(r^2+2y^2) + 2p_2xy]\\
where\ r^2 = x^2 + y^2 and\ p_1,p_2\ to\ be\ the \ coefficients
$$
[reference in csdn](https://blog.csdn.net/u010128736/article/details/52851250)

if given the distortion coefficients with the 2D image point, we could get two binary quadratic equations. To get (x', y'), we could solve the two equations by some means. Finally, we could get 4 sets of (x', y') points. Since the origin is at the corner and starts from (0, 0), the all-positive point should be the coordinate before distortion.

### P5

After transforming into the correct coordinates, the calibration firstly extracts some feature points of the calibration picture. Then they are used to perform certain algorithm to calculate or approach the camera intrinsic.

### P6&P7

[Codes in my Github](https://github.com/SunnyScenery/SomeSol)

[reference: tutorials of opencv-python](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration)

##record

### W1D1

Camera resection: 

3D part: world coordinate and camera coordinate(perspective projection)

$$\left[ \begin{matrix}R&T \\&1\end{matrix} \right] \tag{world coordinate}$$

$$\lambda\left[ \begin{matrix} 1&&& \\&1&&\\&&1&\end{matrix} \right] \tag{camera}$$

2D part: image coordinate system 

$$\left[ \begin{matrix}m_x&&\\&m_y&\\&&1\end{matrix} \right] \left[ \begin{matrix}f&&p_x \\&f&p_y\\&&1\end{matrix} \right] = \left[ \begin{matrix}\alpha_x&&\beta_x \\&\alpha_y&\beta_y\\&&1\end{matrix} \right] $$ 

graphs that help are in the slides from cmu cv class.

> ref: [camera_resection1](https://mixedreality.fandom.com/wiki/Camera_resectioning) and [camera resection2](http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html) and the best one, [slides from cmu computer vision course](http://www.cs.cmu.edu/~16385/s17/Slides/11.1_Camera_matrix.pdf)

*optical* (or lens or camera) *center*

### W1D3

At first, I missed the similarity relation, and by solving the equation it seem there are unique xyz. But by imagination, it should be a ray. After reading this one, it is clear.

[a clearly introduce in zhihu](https://zhuanlan.zhihu.com/p/30813733)

![image-20190529195826930](/Users/scenery/Library/Application Support/typora-user-images/image-20190529195826930.png)

### W1D4

[source1 about distortion in csdn](https://blog.csdn.net/piaoxuezhong/article/details/75268535)

[source2 about distortion in csdn](https://blog.csdn.net/yangdashi888/article/details/51356385)

[source3 about the whole procedure in csdn](https://blog.csdn.net/honyniu/article/details/51004397)

opencv uses the rotation vectors. [additional material about rotation vectors in csdn](https://blog.csdn.net/mightbxg/article/details/79363699)

left09.jpg and left11.jpg: cv2.findChessboardCorners() could not find the corner of these two pictures

in the Tutorial of opencv-python, it introduces the process of camera calibration and undistortion. There are several questions.

- [] the return value parameter (one of the output of calibrateCamera): the meaning of this value remain unknown

- the use of cv2.getOptimalNewCameraMatrix()
  after doing some tests and with documents, I slightly know the meaning of this function.

  <figure class = "third">
    <img src = "/Users/scenery/Code/SomeGit/Stereo/Problems/stereo/test/calibresult1.png"width = 200/><img src = "/Users/scenery/Code/SomeGit/Stereo/Problems/stereo/test/calibresult2.png"width = 200/><img src = "/Users/scenery/Code/SomeGit/Stereo/Problems/stereo/test/calibresult3.png"width = 200/>
  </figure>

  the first image is with getOptimalNewCameraMatrix() function and ROI crop

  the second one is with getOptimalNewCameraMatrix() function and no ROI crop

  the third one is with no getOptimalNewCameraMatrix() function and no ROI crop

  Comparing figure1 and figure2, we can see that it crop the black pixels

  Comparing figure2 and figure3, we can see that if we use the old camera matrix, the image plane is closer and the region the camera see is smaller. We could think it that by deformation, we are moving some parts closer (or farther), (in some ways).

### W1D5

[Homography matrix /blogs](https://www.cnblogs.com/wangguchangqing/p/8287585.html)

[Zhang's method /blogs](https://www.cnblogs.com/wangguchangqing/p/8335131.html)

questions remained for the geometric interpretation part

[absolute conic(left)](https://blog.csdn.net/YhL_Leo/article/details/49357087)

(好多说是方法的详解,其实就只是个翻译)

思考了很久标定时用的3D points怎么得到了,最后在[这篇博客](https://blog.csdn.net/weixin_41695564/article/details/80422329)上找到了比较直观的解释:OpenCV使用棋盘格板进行标定，如下图所示。为了标定相机，我们需要输入一系列三维点和它们对应的二维图像点。在黑白相间的棋盘格上，二维图像点很容易通过角点检测找到。而对于真实世界中的三维点呢？由于我们采集中，是将相机放在一个地方，而将棋盘格定标板进行移动变换不同的位置，然后对其进行拍摄。所以我们需要知道(X,Y,Z)的值。但是简单来说，我们定义棋盘格所在平面为XY平面，即Z=0。对于定标板来说，我们可以知道棋盘格的方块尺寸，例如30mm，这样我们就可以把棋盘格上的角点坐标定义为(0,0,0)，(30,0,0)，(60,0,0)，···，这个结果的单位是mm。

### W1D6

最近学习状态不太对..进度拖后了..

大致流程: 求单应矩阵，求相机内参，求每幅图相应的外参，求畸变矫正系数，微调所有参数

实现参照:[Burger-CameraCalibration-20160516.pdf](http://staff.fh-hagenberg.at/burger/)

..比想象中的要难

### W1D7

继续完成Zhang's method.