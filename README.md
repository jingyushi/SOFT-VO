## SOFT SLAM

This repository is C++ OpenCV implementation of SOFT (Stereo Odometry based on careful Feature selection and Tracking): [PDF](https://lamor.fer.hr/images/50020776/Cvisic2017.pdf)
The usage of each .h file should be found at the tests folder.
Note the repo has not been finished.

### Requirements
[OpenCV 3.0](https://opencv.org/)

[Eigen 3](https://eigen.tuxfamily.org/dox/GettingStarted.html)

### Dataset
Tested on [KITTI](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) odometry dataset

### Compile & Run
```bash
git clone https://github.gatech.edu/swang736/Soft_slam_hahaha.git

```bash
mkdir build
cd build
cmake ..
make -j4
./run /PathtoKITTI/sequences/00/ ../calibration/kitti00.yaml
```
### Reference code

1. [Matlab implementation of SOFT](https://github.com/Mayankm96/Stereo-Odometry-SOFT/blob/master/README.md)
