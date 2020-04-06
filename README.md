
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

## 3. Hardware setup

Below is a list of the hardware you will need to get started:

- Computer with Ubuntu 16.04 and ROS Kinetic
- Teensy 3.2 board
- RGBW led strip
- Wires


## 4. Usage

1. Connect the ALS module to the USB port of the computer. All the leds should start to blink white, this means that the ALS module has started.

2. In a terminal, launch the driver. All the LEDs will turn green when the driver has finished loading. This means that the driver is ready to accept commands.

```
$ roslaunch robotnik_leds_sdk leds_driver.launch robot_name:=example
```

3. In another terminal, send a command to the ALS module. All LEDs will blink green at 1 Hz.

```
$ rosservice call /led_command_interface/command "state: 'MOVING' enable: true"
```

To erase the effect on the strip, disable it.

```
$ rosservice call /led_command_interface/command "state: 'MOVING' enable: false"
```

4. To close the driver, just do CTRL + C. All the LEDs will light up blue. This means that the driver has ended. If the driver is launched again the LEDs will light up green waiting for a command

