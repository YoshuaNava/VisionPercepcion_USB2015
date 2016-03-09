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

**1)** Open a terminal and change directory to the catkin workspace root folder ("VisionPercepcion_USB2015/Code/Thesis_ws").

**2)** To run the EGBIS superpixel segmentation test, type:

```rosrun usbgm_pfs test_superpixel_segmentation ```

**3)** To run an histogram comparison routine for superpixels, type:

```rosrun usbgm_pfs test_histogram_superpixel_comparison ```

**4)** To run the original version of the GPS algorithm by Sapienza et al, type:

```rosrun gps_sapienza_ros gps_sapienza ```

**5)** To run the re-implementation of the GPS algorithm by Sapienza et al, type:

```rosrun usbgm_pfs test_sapienza_full ```

**6)** To run the Hough Horizon search algorithm, enter:

```rosrun usbgm_pfs test_encapsulated_horizon_search ```



## Dependencies:
Eigen, OpenCV 2.4+, ROS índigo igloo, roscpp, cmake 2.8+, gcc, g++, and make.



## Contributions:

**1) gps_sapienza_ros** (working): A ROS node that wraps the original GPS algorithm by Michael Sapienza.

**2) usbgm_pfs**: A ROS packages to wrap the following libraries:

**2.1) libsuperpixels** (working): A library for handling superpixels, that can be found in the folder usbgm_pfs/include/libsuperpixel.

**2.2) slid_modified and egbis** (working): Modified versions of the original SLIC and EGBIS libraries for superpixel segmentation, so that they can be compliant with libsuperpixel.

**2.3) hough_horizon_search** (working): A novel algorithm for identifying the floor using the structure of the scene.

**2.4) gps_sapienza** (working): Re-implementation of the original GPS algorithm by Michael Sapienza as a C++ library, using Modern OpenCV methods.

**2.5) gps_sapienza_encapsulated** (in early stage of development): Re-implementation of the original GPS algorithm by Michael Sapienza, introducing genericity to image features generation and superpixels histogram comparison.



## Acknowledgements

To Professor José Cappelletto, supervisor of this work.

To Michael Sapienza, author of the dataset that is provided by default with this package ("eng_stat_obst.avi"). We have greatly inspired on his work.

To the authors of the SLIC algorithm and its OpenCV wrapper (Achanta, Shaji, et al. And GitHub user PSMM).

To the authors of the EGBIS algorithm and its OpenCV wrapper (Felzenswalb, Huttenlocher, and Michael Sapienza).


## TODO:

Make everything compliant with the ROS C++ Style Guide ( http://wiki.ros.org/CppStyleGuide )

Improve the performance of the SLIC algorithm, using intrinsics and Intel TBB.

Introduce a generic approach to superpixels comparison from different image features to Michael Sapienza algorithm, to merge it with hough_horizon_search and other feature generators.

