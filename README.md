# Stereo Problem

### requirements

- cv2 (opencv-python)
- python3.7
- numpy

### P6P7.py

for running this, change the root path to your path. They are path of origin images, path of feature points on images and path of undistorted images.

```python
python3 P6P7.py
```

###Zhang's method of camera calibration

####in folder zhang/

use

```python
python calibration.py --imgpath 'your-path-to-images'
```

to run,

the output up till now will be

```
'which image does not contain enough feature points'
'camera matrix'
'rotation matrix'
'lambda, the scaler factor'
'translation vector'
```

It will also create a directory called 'corner'. Images with corner points are saved in this folder.

this implementation still remains more efforts

####in jupyter/testForZhang.ipynb

This is used for some tests when implementing.

### P12

for running this, use

```python
python P12.py --imgpath 'your-path-to-images'
```

the path should contain two folders, 'left' and 'right'