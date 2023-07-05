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



## Setup

In order to be able to run the driver as an unprivileged user, you need to set permissions for the camera device. This can be done automatically by copying the [`config/99-seekthermal.rules`](config/99-seekthermal.rules) file from the repository to `/etc/udev/rules.d` and running the following commands:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The user accessing the camera must be a member of `video` group.


## How to correct installation issues :
- I can’t build OpenCV3
If you installed the stereo camera package, you installed CUDA. This software layer can cause you
some troubles with OpenCV3 installation. To prevent it, you will need to add -DWITH CUDA = OFF
when you are building your package. OpenCV3 will now be installed without using CUDA but it’s not
mandatory and your package will work without it.

- libjasper-dev is not found on my computer
When you will install OpenCV3 you may have problems with the library libjasper-dev. If it’s the case,
use this link to find solutions to your problem https://askubuntu.com/questions/1145811/
error-installing-libjasper-dev-and-libpng12-package-in-ubuntu-19-04?rq=1


- Even with this change I can’t build it
Another problem we had when building OpenCV3 was the make command line which crashed dur-
ing the process because we used too many threads. The computer we used had 24 threads, we had to only
use 4 threads to make it compile properly: make -j4

- I have a Cmakelist.txt problem
You may also have a problem when compiling your seekthermal camera package because of your Cmake-
list.txt. To prevent it, navigate to your package ¿ vendor ¿ libseekthermal lite ¿ Cmakelist.txt. Open this
file and replace the line find packageOpenCV 3 REQUIRED COMPONENTS by find packageOpenCV
3 REQUIRED


## Launch the node
```
source install/setup.bash
ros2 launch seekthermal_camera seekthermal_camera.launch.py
```
