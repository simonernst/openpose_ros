# openpose_ros

Example ROS catkin package that utilizes the OpenPose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose.

## For OpenCV (2/3) devices, please see the wiki on the [master](https://github.com/simonernst/openpose_ros/tree/master) branch


## Installation Steps

### If Openpose is already installed on your system, jump to step 3

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

   3.1. Dependencies
   
   ```bash
   sudo apt-get install libboost-dev libboost-all-dev

   sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev libatlas-base-dev liblmdb-dev libblas-dev libatlas-base-dev libprotobuf-dev libleveldb-dev libsnappy-dev libhdf5-serial-dev protobuf-compiler
   ```
   
   3.2. OpenPose
   
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

   3.3. Test
   
   From your openpose directory, you can try an example :
   ```bash
   ./build/examples/openpose/openpose.bin --video examples/media/video.avi
   ```

4. Openpose_ros installation

   **The following steps are in a catkin workspace**
   
   4.1. Clone the repo
   
   ```bash
   git clone https://github.com/simonernst/openpose_ros.git
   cd openpose && git checkout opencv4_compatibility`
   ```
   4.2. Clone vision_opencv
   
   ```bash
   git clone https://github.com/ros-perception/vision_opencv
   ```
      #### checkout the branch (default is for ROS neotic) !!!! 
      `git checkout melodic` (for Ubuntu 18) or `git checkout kinetic` for Ubuntu 16
   
   4.3. Ensure opencv4 compatibility with vision_opencv repo
   
   * Add set (CMAKE_CXX_STANDARD 11) to your top level cmake
   * Remove in cv_bridge/CMakeLists.txt line 16 the number "3" (or change it with 4)
   * In cv_bridge/src CMakeLists.txt line 35 change to if (OpenCV_VERSION_MAJOR VERSION_EQUAL 4)
   * In cv_bridge/src/module_opencv3.cpp change signature of two functions :
   
      * `UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, int flags, UMatUsageFlags usageFlags) const`    **to**    `UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, AccessFlag flags, UMatUsageFlags usageFlags) const`
      
      * `bool allocate(UMatData* u, int accessFlags, UMatUsageFlags usageFlags) const`    **to**   `bool allocate(UMatData* u, AccessFlag accessFlags, UMatUsageFlags usageFlags) const`
   
   4.4. Modify the model_folder line in openpose_ros/src/openpose_flags.cpp to where openpose is installed (line 30)
   
   ```bash
   DEFINE_string(model_folder,             "/path/to/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
   ```
   4.5 Modify the image_topic parameter in openpose_ros/launch/openpose_ros.launch to the image_topic you want to process
   
   ```bash
   <param name="image_topic"     value="/camera/image_raw" />
   ```
   4.6. Modify the other parameters in openpose_ros/src/openpose_flags.cpp and openpose_ros/launch/openpose_ros.launch to your liking such as enabling face and hands detection

   4.7. Build with `catkin_make` from your catkin_workspace directory.


## Running
```bash
source catkin_workspace/devel/setup.bash
roslaunch openpose_ros openpose_ros.launch
```
