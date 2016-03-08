# Probabilistic Floor Segmentation

This repository stores a novel version of an algorithm to segment floor areas in grayscale and RGB images. Its long term purpose is to be part of a
future navigation system for a mobile robot.


Authors:
	Yoshua Nava and Rafael Colmenares

## How to compile the code:

**1)** Open a terminal and change directory to this repository root folder ("VisionPercepcion_USB2015/").

**2)** Type the following commands to compile everything:

```
./setup_everything_ws.sh
bash
```

## Instructions for running the algorithms:

**3)** To run the EGBIS superpixel segmentation test, type:

```./test_egbis_superpixel_segmentation ```

**4)** To run the SLIC superpixel segmentation test, type:

```./test_slic_superpixel_segmentation ```

**5)** To run the superpixel histogram comparison test, type:

```./test_histogram_superpixel_comparison ```


If you want to change the algorithm that is used for generating and comparing superpixels, change the value of the constant SUPERPIXEL_ALGORITHM (line 18) for either "EGBIS" or "SLIC"


## Dependencies:
Eigen, OpenCV 2.4+, cmake 2.8+, gcc, g++, and make.



## Contributions:

gps_sapienza_ros: A ROS node that wraps the original GPS algorithm by Michael Sapienza.

libsuperpixels: A library for handling superpixels, that can be found in the folder usbgm_pfs/include/libsuperpixel.

slid_modified and egbis: Modified versions of the original SLIC and EGBIS libraries for superpixel segmentation, so that they can be compliant with libsuperpixel.

hough_horizon_search: A novel algorithm for identifying the floor using the structure of the scene.

gps_sapienza and gps_sapienza_encapsulated: Re-implementations of the original GPS algorithm by Michael Sapienza, introducing (1) Modern OpenCV methods, and (2) Genericity to image features generation and superpixels histogram comparison.

usbgm_pfs: A ROS packages that wraps all the aforementioned algorithms as libraries.



## Acknowledgements

To Professor José Cappelletto and Rafael Colmenares, supervisors of this work.

To Michael Sapienza, author of the dataset that is provided by default with this package ("eng_stat_obst.avi"). We have greatly inspired on his work.

To the authors of the SLIC algorithm and its OpenCV wrapper (Achanta, Shaji, et al. And GitHub user PSMM).

To the authors of the EGBIS algorithm and its OpenCV wrapper (Felzenswalb, Huttenlocher, and Michael Sapienza).


## TODO:

Make everything compliant with the ROS C++ Style Guide ( http://wiki.ros.org/CppStyleGuide )

Improve the performance of the SLIC algorithm, using intrinsics and Intel TBB.

Introduce a generic approach to superpixels comparison from different image features to Michael Sapienza algorithm, to merge it with hough_horizon_search and other feature generators.

