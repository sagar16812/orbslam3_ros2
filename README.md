# orbslam3_ros2: ORB-SLAM3 ROS 2 Integration

## Overview
The motivation behind creating this package is to support beginners who are just getting started with computer vision and robotics. It is designed for those who want to explore how robots perceive and understand their surroundings using only a basic camera sensor.

This package is especially useful for learners and hobbyists who may not have access to expensive hardware like LiDAR or high-performance computing systems. By wrapping ORB-SLAM3 for ROS 2, this package makes it easier to run real-time SLAM on affordable setups, allowing users to visualise camera motion, build maps, and understand SLAM concepts in a practical and accessible way.

**<center>Rviz2 visualization of `TUM/RGB-D_SLAM_Dataset_and_Benchmark Dataset, Seq: rgbd_dataset_freiburg1_xyz`.</center>**
![Screenshot](./Screenshot.png?raw=true "Screenshot")
<br><br>

**<center>Below is an oversimplified sequence diagram to get an overview of how it works.</center>**
![Sequence_Diagram](./Sequence_Diagram.png?raw=true "Sequence Diagram")

---

## System Requirements
- Ubuntu 22.04
- ROS 2 humble desktop (In case headless system, `ros-humble-base and ros-humble-perception`)
- OpenCV
- Build Pangolin
- Build ORB_SLAM3

---

## Install Prerequisites

### 1. Install ROS 2 (Skip this step if you've already installed it)
If you have not installed ROS 2 on your system yet, you can follow my Medium article [Getting Started with ROS2: Install and Setup ROS2 Humble on Ubuntu 22.04(LTS)](https://medium.com/spinor/getting-started-with-ros2-install-and-setup-ros2-humble-on-ubuntu-22-04-lts-ad718d4a3ac2) or refer to this [repo](https://github.com/sagar16812/Getting-Started-with-ROS2-A-Tutorial-Series).

### 2. Install Required Dependencies
Once you’ve got ROS2 Humble Desktop/Base installed, make sure the OpenCV and cv_bridge package are installed, if not then you can install them by running:
```bash
sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

### 3. Install Pangolin
- Download the Pangolin source code:
    ```bash
    cd
    git clone https://github.com/stevenlovegrove/Pangolin Pangolin
    ```
- Navigate to the downloaded folder and install the recommended prerequisites:
    ```bash
    cd Pangolin
    ./scripts/install_prerequisites.sh recommended
    ```
- Switch to an older version of Pangolin, specifically **`v0.6`**, as it is compatible with ORB-SLAM3.
    ```bash
    git checkout v0.6
    ```
- Before building the library, you must modify a file for compatibility. Add the following line in the file **`Pangolin/include/pangolin/gl/colour.h`**:
    ```cpp
    // Pangolin/include/pangolin/gl/colour.h
    ...

    #include <limits>

    ...
    ```
- Build and Installation:
    ```bash
    mkdir build
    cd build
    cmake .. && make
    make install
    ```
### 4. Install ORB-SLAM3
- Download the ORB-SLAM3 source code:
    ```bash
    git clone -b c++14_comp https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
    ```
- **(Optional)** You can update the `ORB_SLAM3/build.sh` file to limit the number of CPU cores used during the compilation process. In the `ORB_SLAM3/build.sh`file, locate the command used to make that particular library. Reduce the value after `-j` to reduce the number of cores.
- Then build:
    ```bash
    cd ORB_SLAM3
    chmod +x build.sh
    ./build.sh
- After the installation is complete, update your `.bashrc` file to ensure the system can find the ORB-SLAM3 libraries. Run the following command in your terminal, make sure to replace `<path_to_ORB_SLAM3/lib>` with the actual path where you’ve installed ORB-SLAM3:
    ```bash
    echo 'export LD_LIBRARY_PATH=<path to ORB_SLAM3/lib>:/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
    ```
### 5. Install Octomap Server
```bash
sudo apt install ros-humble-octomap-*
```

---

## How to build the package:
1. Clone repository to your ROS workspace
    ```bash
    $ mkdir -p colcon_ws/src
    $ cd ~/colcon_ws/src
    $ git clone https://github.com/sagar16812/orbslam3_ros2.git
    ```
2. Change this [line](https://github.com/sagar16812/orbslam3_ros2/blob/main/CMakeLists.txt?plain=1#L32) to your own `ORB_SLAM3` path in the `CMakeLists.txt` file. 
3. Change this [line](https://github.com/sagar16812/orbslam3_ros2/blob/main/CMakeLists.txt?plain=1#L33) to your own `Pangolin` path in the `CMakeLists.txt` file.
4. Now build
    ```bash
    cd ~/colcon_ws
    colcon build --symlink-install --packages-select orbslam3_ros2
    ```
---

## Troubleshootings
If you cannot find `sophus/se3.hpp`:
- Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
    ```bash
    cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
    sudo make install
    ```

---

## Running `orbslam3_ros2` nodes
This package supports running ORB-SLAM3 with **monocular**, **stereo**, and **RGB-D** input using either a dataset or a live camera stream.

### Prerequisites
Before running, make sure you:
- Copy the `<ORB_SLAM3_ROOT_DIR>/ORB_SLAM3/Vocabulary/ORBvoc.txt` to the `config/` directory (`~/colcon_ws/src/orbslam3_ros2/config`).
- Configure the camera parameters YAML config files under the `config/` directory based on your `camera type` and calibration result.
- Source the workspace:
    ```bash
    cd ~/colcon_ws
    source install/setup.bash
    ```
- Start the image publishing node (you can use the `v4lcamera` package)

### Launch using the launch file with relevant `camera_type`
1. Monocular executable: `mono`
    ```bash
    ros2 launch orbslam3_ros2 orbslam3_ros2.launch.py camera_type:=mono
    ```
2. Stereo executable: `stereo`
    ```bash
    ros2 launch orbslam3_ros2 orbslam3_ros2.launch.py camera_type:=stereo
    ```
3. RGB-D executable: `rgbd`
    ```bash
    ros2 launch orbslam3_ros2 orbslam3_ros2.launch.py camera_type:=rgbd
    ```

> Make sure the **Image publishing node** is publishing images on the expected topics as configured in the nodes of respective executable.

## Options
This package has options to visualize or use octomap_server or save rosbag. We can use all the options at the same time.
- **RViz2 Visualization**: Load the `orbslam3_ros2` with pre-configured `config/orbslam3_ros2.rviz` file:
    ```bash
    ros2 launch orbslam3_ros2 orbslam3_ros2.launch.py camera_type:=mono visualize:=true
    ```
- **Start octomap_server**: 
    ```bash
    ros2 launch orbslam3_ros2 orbslam3_ros2.launch.py camera_type:=mono start_octomap:=true
    ```
- **Record a rosbag of all the topics**:
    ```bash
    ros2 launch orbslam3_ros2 orbslam3_ros2.launch.py camera_type:=mono record_bag:=true
    ```

## Project Structure

```bash
orbslam3_ros2/
├── CMakeLists.txt                     # CMake build configuration
├── package.xml                        # ROS 2 package metadata
├── README.md                          # Project documentation

├── config/                            # Configuration files
│   ├── ORBvoc.txt                     # ORB-SLAM3 vocabulary file
│   ├── orbslam3_ros2.rviz             # RViz2 configuration file
│   ├── camera_and_slam_settings.yaml  # Default
│   ├── stereo_camera_config.yaml      
│   ├── TUM_RGB-D_Dataset.yaml         
│   └── FinnForest_stereo.yaml         

├── launch/                            # ROS 2 launch files
│   └── orbslam3_ros2.launch.py        # Unified launch file for mono, stereo, rgbd

├── include/orbslam3_ros2/             # C++ headers
│   ├── image_grabber_mono.hpp         # Monocular input handler
│   ├── image_grabber_stereo.hpp       # Stereo input handler
│   └── image_grabber_rgbd.hpp         # RGB-D input handler

├── src/                               # C++ source files
│   ├── orb_slam_mono.cpp              # Mono SLAM integration
│   ├── orb_slam_stereo.cpp            # Stereo SLAM integration
│   ├── orb_slam_rgbd.cpp              # RGB-D SLAM integration
│   ├── image_grabber_mono.cpp         # Mono image subscriber
│   ├── image_grabber_stereo.cpp       # Stereo image subscriber
│   └── image_grabber_rgbd.cpp         # RGB-D image subscriber
```

### Notes:
- **`image_grabber_*.cpp`** files handle image processing, data conversion, and data publishing.
- **`orb_slam_*.cpp`** files initialize and run the ORB-SLAM3 system.
- Configuration files are modular, so you can easily swap between datasets or camera types.

---

## References
1. [Integrating ORB-SLAM3 with ROS2 Humble on Raspberry Pi 5: A Step-by-Step Guide](https://medium.com/@antonioconsiglio/)integrating-orb-slam3-with-ros2-humble-on-raspberry-pi-5-a-step-by-step-guide-78e7b911c361
2. https://github.com/zang09/ORB_SLAM3_ROS2?tab=readme-ov-file
3. https://github.com/zang09/ORB-SLAM3-STEREO-FIXED
4. My Article Series: [Getting Started with ROS2](https://medium.com/@sagarcadet/list/getting-started-with-ros2-adb24ab6d8dd)
5. Github: [Getting-Started-with-ROS2-A-Tutorial-Series](https://github.com/sagar16812/Getting-Started-with-ROS2-A-Tutorial-Series)
---

## License
This repository is licensed under the GNU General Public License v3.0. See the LICENSE file for more details.

---