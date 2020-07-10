# openpose_ros

Example ROS catkin package that utilizes the OpenPose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose.

## For Opencv4 devices (e.g Jetson Xavier), please see the wiki on the [opencv4_compatibility](https://github.com/simonernst/openpose_ros/tree/opencv4_compatibility) branch


## Installation Steps

1. Clone OpenPose somewhere not in your catkin_workspace.
   ```bash
   git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
   ```
   
2. Cmake upgrade (only if `cmake -V`show < 3.12.2)
   ```bash
   wget http://www.cmake.org/files/v3.12/cmake-3.12.2.tar.gz
   tar -xvzf cmake-3.12.2.tar.gz 
   cd cmake-3.12.2/
   ./configure 
   make
   ```
   
3. Install OpenPose

   3.1 Dependencies
   ```bash
   sudo apt-get install libboost-dev libboost-all-dev

   sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev libatlas-base-dev liblmdb-dev libblas-dev libatlas-base-dev libprotobuf-dev libleveldb-dev libsnappy-dev libhdf5-serial-dev protobuf-compiler
   ```
   
   3.2 OpenPose
   ```bash
   cd openpose
   cd models && sh getModels.sh && cd ..
   mkdir build && cd build && cmake ..
   make -j`nproc`
   sudo make install
    ```
Make sure to run `sudo make install` in the build folder at the end.    
    
More OpenPose instructions for specific installations can be found here :
https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/254570df262d91b1940aaf5797ba6c5d6db4b52f/doc/installation.md. 

   3.3 Test
   From your openpose directory, you can try an example :
   ```bash
   ./build/examples/openpose/openpose.bin --video examples/media/video.avi
   ```

4. Openpose_ros installation

**The following steps are in a catkin workspace**

   4.1 Clone the repo
   ```bash
   git clone https://github.com/simonernst/openpose_ros.git
   ```
   
   4.2 Modify the model_folder line in openpose_ros/src/openpose_flags.cpp to where openpose is installed (line 30).
   ```bash
   DEFINE_string(model_folder,             "/path/to/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
   ```
   4.3 Modify the image_topic parameter in openpose_ros/launch/openpose_ros.launch to the image_topic you want to process.
   ```bash
   <param name="image_topic"     value="/camera/image_raw" />
   ```
   4.4 Modify the other parameters in openpose_ros/src/openpose_flags.cpp and openpose_ros/launch/openpose_ros.launch to your liking such as enabling face and hands detection.
   
   4.5 Run `catkin_make` from your catkin_workspace directory.


## Running
```bash
source catkin_workspace/devel/setup.bash
roslaunch openpose_ros openpose_ros.launch
```
