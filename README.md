
## 1. Overview

## 2. Installation

1. Create and select the working directory


```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/devel/setup.bash
```

2. Clone the repository in the working directory

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/robert-ros/robotnik_leds.git
```

3. Install the rosserial dependency, this will allow communication between ROS and the ALS module

```
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-rosserial-arduino
$ sudo apt-get install ros-kinetic-rosserial
```

4. Install the rest of dependencies

```
$ cd ~/catkin_ws/
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y
```

5. Build the working directory and messages

```
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/devel/setup.bash
```

6. Add udev rule from ALS module and reload and trigger the updated udev rules

```
$ sudo cp ~/catkin_ws/src/robotnik_leds/robotnik_leds_utils/49-teensy-leds.rules /etc/udev/rules.d
$ sudo udevadm control --reload-rules && udevadm trigger
```

7. Connect the ALS module via USB. Verify that the port is shown under the ``` /dev/ttyLEDs ``` symlink

```
cd /dev && ls -l ttyLEDs
```
