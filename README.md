
# ArUco Marker Detection

## 1. Environment Setup
Generation scripts require Python libraries. It's recommended to use a virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate

pip install opencv-contrib-python numpy Pillow
```

## 2. Camera Calibration (ROS 1)
1. **Install Tool**:
   ```bash
   sudo apt install ros-noetic-camera-calibration
   ```
2. **Run Calibration** (Use **Calibration Board**):

   [Official Docs](https://wiki.ros.org/camera_calibration)
   ```bash
   rosrun camera_calibration cameracalibrator.py --size 4x6 --square 0.035 image:=/camera/image_raw camera:=/camera
   ```
3. **Save**:
   Commit calibration. It saves to `~/.ros/camera_info/head_camera.yaml`.

## 3. ArUco Detection
1. **Install Package**:
   ```bash
   sudo apt install ros-noetic-aruco-ros
   ```
2. **Launch** (Use **Tracking Marker**):
   ```bash
   roslaunch launch/aruco_detect.launch
   ```
   *Note*: This launch file is configured for Marker ID 26 with size 0.15m (15cm).

## 4. MAVROS Bridge
1. **Run Bridge**:
   ```bash
   python3 scripts/aruco_mavros_bridge.py
   ```
   Publishes `/mavros/vision_pose/pose`.
