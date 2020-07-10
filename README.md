# openpose_ros

Example ROS catkin package that utilizes the OpenPose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose.

## System
Tested on:
* Ubuntu 14.04 / Ubuntu 16.04
* ROS Indigo / Kinetic
* CUDA 8.0 / CUDA 10.0 / CUDA 10.1
* cuDNN 5.1 / cuDNN 6.0 / cuDNN 7.2.4 / cuDNN 7.5.0
* OpenCV 3.3 / OpenCV 3.4

## Installation Steps

1. Clone OpenPose somewhere not in your catkin_workspace.
   ```bash
   git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
   ```
2. Install OpenPose
   ```bash
   cd openpose
   mkdir build && cd build
   cmake ..
   make -j`nproc`
    ```
Make sure to run `sudo make install` in the build folder at the end.    
    
More OpenPose instructions for specific installations can be found here :
https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/254570df262d91b1940aaf5797ba6c5d6db4b52f/doc/installation.md. 

3. Clone this repository into your catkin_workspace/src directory.
   ```bash
   git clone https://github.com/simonernst/openpose_ros.git
   ```
   3.1
   If your are using a device with OpenCv4 (e.g Jetson Xavier), `git checkout opencv4_compatibility`
   
4. Modify the model_folder line in openpose_ros/src/openpose_flags.cpp to where openpose is installed (line 30).
   ```bash
   DEFINE_string(model_folder,             "/path/to/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
   ```
5. Modify the image_topic parameter in openpose_ros/launch/openpose_ros.launch to the image_topic you want to process.
   ```bash
   <param name="image_topic"     value="/camera/image_raw" />
   ```
6. Modify the other parameters in openpose_ros/src/openpose_flags.cpp and openpose_ros/launch/openpose_ros.launch to your liking such as enabling face and hands detection.
7. Run catkin_make from your catkin_workspace directory.

### Potential Installation Issues
1. If cv_bridge is causing you errors and/or you decide to use OpenCV 3.2+, copy the cv_bridge folder from https://github.com/ros-perception/vision_opencv into your catkin_workspace/src directory. 

   1.1 Check the branch your are on when cloning (default is neotic). `git checkout melodic` for Ubuntu 18 or `git checkout kinetic` for Ubuntu 16

## Running
```bash
source catkin_workspace/devel/setup.bash
roslaunch openpose_ros openpose_ros.launch
```
