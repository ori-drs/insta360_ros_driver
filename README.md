# CHANGES to previous... 
- Driver is tested on Ubuntu 24.04 with ROS2 Jazzy using Insta360 RS-1inch and X5 cameras.
- Resolution: 3840x1920, can be changed in main.cpp
- Requires SDK. DO NOT add the SDK to the REPO, due to copyright! 
- Deactivated imu_tools from IMU meas
- Timesync with ROS: averaging offset over last N messages
- Timestamping for IMU messages added.
- Timestamping for Images retained during decoding ( assumes single compressed for each ).
- Caveat: raw sdk stamps for IMU and Camera seem to diverge over time (IMU runs ahead after sometime by mutiple seconds), but frames appear consistent.
- Split launch file to separate for capturing / processing (e.g. capture on robot w/o decoding, process offboard)
- Added camera_info for both images.
- Added publishing of individual fisheye images for both cameras.

# insta360_ros_driver

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 22.04 with ROS2 Humble. The driver has also been verified on the Insta360 X2 and X3 cameras. The following resolutions are available, all at 30 FPS.
- 3840 x 1920
- 2560 x 1280
- 2304 x 1152
- 1920 x 960


## Installation
To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the [Insta360 website](https://www.insta360.com/sdk/home). 

For additional instructions, see this [post](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

**Note: Please make you use the latest SDK. This package works with the SDK posted after April 23, 2025**

```
cd ~/ros2_ws/src
git clone https://github.com/ori-drs/insta360_ros_driver
cd ..
```
Then, the Insta360 libraries need to be installed as follows:
- add the <code>camera</code> and <code>stream</code> header files inside the <code>include</code> directory
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

Afterwards, install the other required dependencies and build.
```
colcon build --symlink-install
source install/setup.bash
```

Before continuing, **make sure the camera is set to dual-lens mode**

Additionally, **ensure the camera's USB mode is set to Android**:
1. On the camera, swipe down the screen to the main menu
2. Go to Settings -> General
3. Set USB Mode to **Android** (not Webcam or other modes)
4. This is required for the ROS driver to properly detect and communicate with the camera (see [Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4))

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:
```
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh
```
This creates a symlink  based on the vendor ID of Insta360 cameras. The symlink, in this case <code>/dev/insta</code> is used to grant permissions to the usb port used by the camera.

**Sometimes, this does not work (e.g. you see "device /dev/insta not found" or something similar). You can try entering the commands manually, since that sometimes sees success, especially for the first time.**
```
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

## Usage
The camera provides images natively in H264 compressed image format. 
A driver node connects with the camera to publish the H264 compressed data. 
A separate decoder node can decode this video and publish the dual fisheye image or run without the driver on a separate system.
Both nodes can be run indepently by deactivating them during launch.

```
ros2 launch insta360_ros_driver driver.launch.xml driver:=true decoder:=true
```

## Published Topics
- /insta/dual_fisheye/image
- /insta/dual_fisheye/image/compressed
- /insta/cam0/camera_info
- /insta/cam0/image_raw
- /insta/cam0/image_raw/compressed
- /insta/cam1/camera_info
- /insta/cam1/image_raw
- /insta/cam1/image_raw/compressed
- /insta/imu/data_raw


If you do not care about live processing, you can simply record the `/insta/dual_fisheye/image/compressed` topic and decompress it later after recording.
```
ros2 bag record /insta/dual_fisheye/image /insta/imu/data_raw
```
