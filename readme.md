

# Vision Recon Robot (ROS 2 + Gazebo)

This package implements a robotic vision pipeline for 3D reconstruction:

* Spawns a robot in **Gazebo**.
* Moves the robot in a **circular path** around an object.
* Captures RGB images from the robot’s camera.
* Saves the images for later reconstruction.

---

## 📦 Dependencies

Install required ROS 2 packages (replace `${ROS_DISTRO}` with `humble`, `iron`, etc.):

```bash
sudo apt update
sudo apt install \
  ros-${ROS_DISTRO}-gazebo-ros \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-image-view \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-std-msgs \
  python3-opencv python3-numpy
```

---

## 📂 Project Structure

```
vision_recon/
├── launch/
│   └── full_scan_launch.py        # Launches Gazebo, robot, movement, image saving
├── scripts/
│   ├── move_and_capture.py        # Moves robot in a circle & triggers captures
│   └── camera_saver.py (optional) # Alternative image saver
├── urdf/
│   └── myrobot.xacro              # Robot description (with camera + drive)
├── worlds/
│   └── my_world.world             # Gazebo world
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🚀 Running the System

### 1. Build

```bash
cd ~/try
colcon build
source install/setup.bash
```

### 2. Launch full system

```bash
ros2 launch vision_recon full_scan_launch.py
```

This will:

* Start **Gazebo** with your world.
* Spawn the robot.
* Run `move_and_capture.py` to move in a circle.
* Start image saver.

---

## 📸 Image Saving

Captured images will be stored in:

```
~/try/recon_images/
```

with filenames like:

```
img_0001.jpg, img_0002.jpg, ...
```

### Manual Save Option

Instead of automatic saving, you can run this in another terminal:

```bash
ros2 run image_view image_saver \
  --ros-args -r image:=/camera/image_raw \
  -p save_all_image:=true \
  -p filename_format:=/home/arushi/try/recon_images/img_%04d.jpg
```

---

## 🔧 Customization

* **Change spawn point** → edit `full_scan_launch.py`:

```python
arguments=['-file', xacro_path, '-entity', 'vision_bot',
           '-x','2.0','-y','1.0','-z','0.0','-Y','1.57'],
```

* **Change number of views or radius** → edit params in `move_and_capture.py`:

```python
self.radius = 1.0
self.n_views = 12
```

---

## 🛠 Troubleshooting

* Robot not moving → check your URDF drive plugin.
* Images not saving → confirm topic name matches (`/camera/image_raw`).
* Make scripts executable:

```bash
chmod +x ~/try/src/vision_recon/scripts/move_and_capture.py
chmod +x ~/try/src/vision_recon/scripts/camera_saver.py
```

---
