# seek_thermal_ROS2

This package was orginally for ROS1 and was ported for ROS2.

## How to install everything you need

- Install Open CV3 by entering the first line of command of this file [link](https://gist.githubusercontent.com/syneart/3e6bb68de8b6390d2eb18bff67767dcb/raw/OpenCV3.2withContrib.sh?fbclid=IwAR2uYKMEiVSCMom-KfYFPUw2ZbwvKziqvv-Y6mto9rkrnG6Btq1Cjrf_Plc)

- Download libusb with this command line : sudo apt install libusb-1.0-0-dev

- Add the library seek_thermal and the ROS2 package seekthermal_camera to the src folder of your ROS2 workspace.
```
cd ~/ros2_ws/src
git clone
```



- Build the library with Cmake
```
source /opt/ros/<distro>/setup.bash
cd ~/ros2_ws/src/seekthermal/build
cmake ..
make
sudo make install
```
- Build the library with colcon to have access to the headers
```
colcon build --packages-select seek_thermal
```
- Build the ROS2 package
```
source install/setup.bash
colcon build --packages-select seekthermal_camera
```
- Launch the package
```
source install/setup.bash
ros2 launch seekthermal_camera seekthermal_camera.launch.py
```


## Setup

In order to be able to run the driver as an unprivileged user, you need to set permissions for the camera device. This can be done automatically by copying the [`config/99-seekthermal.rules`](config/99-seekthermal.rules) file from the repository to `/etc/udev/rules.d` and running the following commands:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The user accessing the camera must be a member of `video` group.



