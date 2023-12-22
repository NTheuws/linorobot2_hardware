## Build status
<!-- Build Status populated by Github Actions runs -->
ROS 2 Distro | Branch | Build status
:----------: | :----: | :----------:
**Rolling** | [`rolling`](../../tree/rolling) | [![Rolling Firmware Build](../../actions/workflows/rolling-firmware-build.yml/badge.svg?branch=rolling)](../../actions/workflows/rolling-firmware-build.yml?branch=rolling)
**Humble** | [`humble`](../../tree/humble) | [![Humble Firmware Build](../../actions/workflows/humble-firmware-build.yml/badge.svg?branch=humble)](../../actions/workflows/humble-firmware-build.yml?branch=humble)
**Galactic** | [`galactic`](../../tree/galactic) | [![Galactic Firmware Build](../../actions/workflows/galactic-firmware-build.yml/badge.svg?branch=galactic)](../../actions/workflows/galactic-firmware-build.yml?branch=galactic)
**Foxy** | [`foxy`](../../tree/foxy) | [![Foxy Firmware Build](../../actions/workflows/foxy-firmware-build.yml/badge.svg?branch=foxy)](../../actions/workflows/foxy-firmware-build.yml?branch=foxy)

## Table of Contents  
- [Building the robot](#building-the-robot)
- [Installation](#installation)
- [Setting up the firmware](#setting-up-the-firmware)
- [Calibration](#calibration)
- [Upload the firmware](#upload-the-firmware)
- [Testing the robot](#testing-the-robot)
- [URDF](#urdf)
- [Troubleshooting Guide](#troubleshooting-guide)
  
## Building the robot

This will list all necessary parts for the robot and further describe the options within each category followed by examples of what the result could look like. 

### 1. Part overview

If you're planning on building a robot, you'll be requiring a set amount of parts. This will give an overview and sort of checklist to see if you haven't missed anything when gathering the hardware.

- Wheels
- Motor drivers
- Motors
- Ineartial Measurement Unit (IMU)
- Laser sensor
- Depth sensor
- Micro controller
- Robot computer
- Battery
- Robot body
- Connection cables

### 1.1 Robot types and orientation
There are 3 different types of robots
- 2 Wheel drive (2WD)
- 4 Wheel drive (4WD)
- Mecanum

Robot Orientation:

-------------FRONT-------------

WHEEL1 WHEEL2 (2WD)

WHEEL3 WHEEL4 (4WD)

--------------BACK--------------

In case you're building a 2WD robot, use the front 2 wheels. Assign `MOTOR1` and `MOTOR2` to the left and right motors respectively. For the robot to drive properly, 1 or 2 caster wheels are used to keep the balance without being in the way of the robot's movement.
For a 4WD and Mecanum robot all 4 motors will be used.

For mecanum robots, the wheels' orientation is very important. When looking at the robot from the top, the rollers on the wheels have to point towards the center of the robot. This should automatically mean the rollers on the bottom of the wheels should form a diamond shape. 

The image below shows the top view of a robot. In case of mecanum wheels, it also shows the orientation of the rollers on the 4 wheels. The arrows show the X and Y-axis for the Inertial Measurement Unit(IMU) make sure these will be alligned later.

![mecanum_wheels_orientation](docs/mecanum_wheels_orientation.png)

### 1.2 Motor Drivers

Supported Motor Drivers:

- **GENERIC_2_IN_MOTOR_DRIVER** - Motor drivers that have EN (pwm) pin, and 2 direction pins (usually DIRA, DIRB pins). Example: L298 Breakout boards.

- **GENERIC_1_IN_MOTOR_DRIVER** - Motor drivers that have EN (pwm) pin, and 1 direction pin (usual DIR pin). These drivers usually have logic gates included to lessen the pins required in controlling the driver. Example: Pololu MC33926 Motor Driver Shield.

- **BTS7960_MOTOR_DRIVER** - BTS7960 motor driver.

- **ESC_MOTOR_DRIVER** - Bi-directional (forward/reverse) electronic speed controllers.

The motor drivers are configurable from the config file explained in the later part of this document.

### 1.3 Motors

The motors will be powered through the motor drivers, this means there are no set requirements besides from them having encoders. Encoders allow the robot to more precisely measure the rotations of the wheels wheels allows the positioning and wayfinding to be more accurate. 
Make sure the motor drivers can handle the amount of voltage and current the motors will need. In most cases each motor needs a driver to be used some drivers however, can handle 2 motors at once. 

### 1.4 Inertial Measurement Unit (IMU)

The IMU is responsible for measuring the speed and direction at which the robot is moving. Together with the motor encoders it'll determine the current place the robot is within the area.
Supported IMUs:

- **GY-85**
- **MPU6050**
- **MPU9150**
- **MPU9250**

### 1.5 Laser sensor

This is the sensor that'll be used to scan the surrouding area of the robot to be able to create a map. The supported options are:

- [RP LIDAR A1](https://www.slamtec.com/en/Lidar/A1)
- [LD06 LIDAR](https://www.inno-maker.com/product/lidar-ld06/)
- [YDLIDAR](https://www.ydlidar.com/lidars.html)
- [XV11](http://xv11hacking.rohbotics.com/mainSpace/home.html)
- *[Intel RealSense](https://www.intelrealsense.com/stereo-depth/) D435, D435i
- *[Zed](https://www.stereolabs.com/zed)
- *[Zed 2](https://www.stereolabs.com/zed-2)
- *[Zed 2i](https://www.stereolabs.com/zed-2i)
- *[Zed Mini](https://www.stereolabs.com/zed-mini) 

Sensors marked with an asterisk are depth sensors. If a depth sensor is used as a laser sensor, the launch files will run [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan) to convert the depth sensor's depth image to laser scans. It is prefered to use an actual laser sensor with a 360° view, meaning the ones listen above without the asterisk.

### 1.6 Depth sensor

This sensor will be mounted to the very front of the robot to give a view of what it's facing. The options that're supported are as follows:

- [Intel RealSense](https://www.intelrealsense.com/stereo-depth/) D435, D435i
- [Zed](https://www.stereolabs.com/zed)
- [Zed 2](https://www.stereolabs.com/zed-2)
- [Zed 2i](https://www.stereolabs.com/zed-2i)
- [Zed Mini](https://www.stereolabs.com/zed-mini)
- [OAK D](https://shop.luxonis.com/collections/oak-cameras-1/products/oak-d)
- [OAK D Lite](https://shop.luxonis.com/collections/oak-cameras-1/products/oak-d-lite-1)
- [OAK-D Pro](https://shop.luxonis.com/collections/oak-cameras-1/products/oak-d-pro)

### 1.7 Micro controller

The micro controller will be responsible for the motors and and IMU. The ones that're supported are:

- Teensy 3.1
- Teensy 3.5
- Teensy 3.6
- Teensy 4.0
- Teensy 4.1

### 1.8 Robot computer

This is what will be running the ROS2 package and uploads the necessary files to the micro controller. Later on a connection will be set up for you to be able to interact with this remotely. The ones that're supported are: 

- Jetson TK1
- Jetson TX1
- Raspberry Pi 3B+
- Odroid XU4
- Radxa Rock Pro

### 1.9 Battery

The battery of the robot has no requirements which means anything can be used as long as it gives enough power for the robot to run. For ease of use, make sure it's either rechargeable or easily accessible to be able to swap it in case it's empty.
In case you're building a robot which doesn't use 5V for the motors it's neccesary to create a way to both power the robot computer and the motors. This can be done with for example a stepdown converter. There's also a posibility to add a switch to the robot, this can then be used to turn the power to the robot computer on and off.

### 1.10 Robot body

For this there are no set requirements either, in essence everyone creates their own variant on the Linorobot2. This means everyone has a different robot designed for their own needs. When it comes to deciding on what to use for the robot's body, it's good to think about the necessary space that's required for the selected hardware. Also something to keep in mind is the ability to attach the motors and sensors to the robot in a way that they're not able to move from their position.


### 2 Connection Diagram
Below are connection diagrams you can follow for each supported motor driver and IMU. For simplicity, only one motor connection is provided but the same diagram can be used to connect the rest of the motors. You are free to decide which microcontroller pin to use just ensure that the following are met:

- Reserve SCL0 and SDA0 (pins 18 and 19 on Teensy boards) for IMU.

- When connecting the motor driver's EN/PWM pin, ensure that the microcontroller pin used is PWM enabled. You can check out PJRC's [pinout page](https://www.pjrc.com/teensy/pinout.html) for more info.

Alternatively, you can also use the pre-defined pin assignments in lino_base_config.h. Teensy 3.x and 4.x have different mapping of PWM pins, read the notes beside each pin assignment in [lino_base_config.h](https://github.com/linorobot/linorobot2_hardware/blob/master/config/lino_base_config.h#L112) carefully to avoid connecting your driver's PWM pin to a non PWM pin on Teensy. 

All diagrams below are based on Teensy 4.0 microcontroller and GY85 IMU. Click the images for higher resolution.

#### 2.1 GENERIC 2 IN

![generic_2_in_connection](docs/generic_2_in_connection.png)

#### 2.2 GENERIC 1 IN

![generic_1_in_connection](docs/generic_1_in_connection.png)

#### 2.3 BTS7960

![bts7960_connection](docs/bts7960_connection.png)

#### 2.4 IMU

![imu_connection](docs/imu_connection.png)

Take note of the IMU's correct orientation when mounted on the robot. Ensure that the IMU's axes are facing the correct direction:

- **X** - Front
- **Y** - Left
- **Z** - Up

#### 2.5 System Diagram
Reference designs you can follow in building your robot.

A minimal setup with a 5V powered robot computer.
![minimal_setup](docs/minimal_setup.png)

A more advanced setup with a 19V powered computer and USB hub connected to sensors.
![advanced_setup](docs/advanced_setup.png)

For bigger robots, you can add an emergency switch in between the motor drivers' power supply and motor drivers.

## Installation
All software mentioned in this guide must be installed on the robot computer.

### 1. Linux

First things first, you want to select a ROS2 distro. The ones that're available and compatible with the robot are shown at the top. [This site](https://docs.ros.org/en/humble/Releases.html) has all the ROS2 distro's shown. Go to the page of the one you want to use, go to supported platforms and pick the one you want to use.

### 2. ROS2 and linorobot2 installation
It is assumed that you already have ROS2 and linorobot2 package installed. If you haven't, go to [linorobot2](https://github.com/linorobot/linorobot2) package for installation guide.

### 3. Download linorobot2_hardware

    cd $HOME
    git clone https://github.com/linorobot/linorobot2_hardware -b $ROS_DISTRO

### 4. Install PlatformIO
Download and install platformio. [Platformio](https://platformio.org/) allows you to develop, configure, and upload the firmware without the Arduino IDE. This means that you can upload the firmware remotely which is ideal on headless setup especially when all components have already been fixed. 
    
    curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 get-platformio.py
    
Add platformio to your $PATH:

    echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
    source $HOME/.bashrc

### 5. UDEV Rule
Download the udev rules from Teensy's website:

    wget https://www.pjrc.com/teensy/00-teensy.rules

and copy the file to /etc/udev/rules.d :

    sudo cp 00-teensy.rules /etc/udev/rules.d/

### 6. Install Screen Terminal

    sudo apt install screen


## Setting up the firmware
### 1. Robot Settings
Go to the config folder and open lino_base_config.h. Uncomment the base, motor driver and IMU you want to use for your robot. For example:

    #define LINO_BASE DIFFERENTIAL_DRIVE
    #define USE_GENERIC_2_IN_MOTOR_DRIVER
    #define USE_GY85_IMU

Constants' Meaning:

*ROBOT TYPE (LINO_BASE)*
- **DIFFERENTIAL_DRIVE** - 2 wheel drive or tracked robots w/ 2 motors.

- **SKID_STEER** - 4 wheel drive robots.

- **MECANUM** - 4 wheel drive robots using mecanum wheels.

*MOTOR DRIVERS*
- **USE_GENERIC_2_IN_MOTOR_DRIVER** - Motor drivers that have EN (pwm) pin, and 2 direction pins (usually DIRA, DIRB pins).

- **USE_GENERIC_1_IN_MOTOR_DRIVER** - Motor drivers that have EN (pwm) pin, and 1 direction pin (usual DIR pin). These drivers usually have logic gates included to lessen the pins required in controlling the driver.

- **USE_BTS7960_MOTOR_DRIVER** - BTS7960 motor driver.

- **USE_ESC_MOTOR_DRIVER** - Bi-directional (forward/reverse) electronic speed controllers.

*INERTIAL MEASUREMENT UNIT (IMU)*
- **USE_GY85_IMU** - GY-85 IMUs.

- **USE_MPU6050_IMU** - MPU6060 IMUs.

- **USE_MPU9150_IMU** - MPU9150 IMUs.

- **USE_MPU9250_IMU** - MPU9250 IMUs.

Next, fill in the robot settings accordingly:

    #define K_P 0.6
    #define K_I 0.8
    #define K_D 0.5

    #define MOTOR_MAX_RPM 100             
    #define MAX_RPM_RATIO 0.85          
    #define MOTOR_OPERATING_VOLTAGE 24
    #define MOTOR_POWER_MAX_VOLTAGE 12
    #define MOTOR_POWER_MEASURED_VOLTAGE 11.7

    #define COUNTS_PER_REV1 2200    
    #define COUNTS_PER_REV2 2200      
    #define COUNTS_PER_REV3 2200      
    #define COUNTS_PER_REV4 2200      
  
    #define WHEEL_DIAMETER 0.09  
    #define LR_WHEELS_DISTANCE 0.2  

    #define PWM_BITS 10
    #define PWM_FREQUENCY 20000

Constants' Meaning:

- **K_P, K_I, K_D** - [PID](https://en.wikipedia.org/wiki/PID_controller) constants used to translate the robot's target velocity to motor speed. These values would likely work on your build, change these only if you experience jittery motions from the robot or you'd want to fine-tune it further.

- **MOTOR_MAX_RPM** - Motor's maximum number of rotations it can do in a minute specified by the manufacturer.

- **MAX_RPM_RATIO** - Percentage of the motor's maximum RPM that the robot is allowed to move. This parameter ensures that the user-defined velocity will not be more than or equal the motor's max RPM, allowing the PID to have ample space to add/subtract RPM values to reach the target velocity. For instance, if your motor's maximum velocity is 0.5 m/s with `MAX_RPM_RATIO` set to 0.85, and you asked the robot to move at 0.5 m/s, the robot's maximum velocity will be capped at 0.425 m/s (0.85 * 0.5m/s). You can set this parameter to 1.0 if your wheels can spin way more than your operational speed.

    Wheel velocity can be computed as:  MAX_WHEEL_VELOCITY = (`MOTOR_MAX_RPM` / 60.0) * PI * `WHEEL_DIAMETER` 

- **MOTOR_OPERATING_VOLTAGE** - Motor's operating voltage specified by the manufacturer (usually 5V/6V, 12V, 24V, 48V). This parameter is used to calculate the motor encoder's `COUNTS_PER_REV` constant during calibration and actual maximum RPM of the motors. For instance, a robot with `MOTOR_OPERATING_VOLTAGE` of 24V with a `MOTOR_POWER_MAX_VOLTAGE` of 12V, will only have half of the manufacturer's specified maximum RPM ((`MOTOR_POWER_MAX_VOLTAGE` / `MOTOR_OPERATING_VOLTAGE`) * `MOTOR_MAX_RPM`). 

- **MOTOR_POWER_MAX_VOLTAGE** - Maximum voltage of the motor's power source. This parameter is used to calculate the actual maximum RPM of the motors.

- **MOTOR_POWER_MEASURED_VOLTAGE** - Measured voltage of the motor's power source. If you don't have a multimeter, it's best to fully charge your battery and set this parameter to your motor's operating voltage (`MOTOR_OPERATING_VOLTAGE`). This parameter is used to calculate the motor encoder's `COUNTS_PER_REV` constant. You can ignore this if you're using the manufacturer's specified counts per rev.

- **COUNTS_PER_REVX** - The total number of pulses the encoder has to read to be considered as one revolution. You can either use the manufacturer's specification or the calibrated value in the next step. If you're planning to use the calibrated value, ensure that you have defined the correct values for `MOTOR_OPERATING_VOLTAGE` and `MOTOR_POWER_MEASURED_VOLTAGE`.

- **WHEEL_DIAMETER** - Diameter of the wheels in meters.

- **LR_WHEELS_DISTANCE** - The distance between the center of left and right wheels in meters.

- **PWM_BITS** - Number of bits in generating the PWM signal. You can use the default value if you're unsure what to put here. More info [here](https://www.pjrc.com/teensy/td_pulse.html).

- **PWM_FREQUENCY** - Frequency of the PWM signals used to control the motor drivers. You can use the default value if you're unsure what to put here. More info [here](https://www.pjrc.com/teensy/td_pulse.html).

### 2. Hardware Pin Assignments
Only modify the pin assignments under the motor driver constant that you are using ie. `#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER`. You can check out PJRC's [pinout page](https://www.pjrc.com/teensy/pinout.html) for each board's pin layout.

The pin assignments found in lino_base_config.h are based on Linorobot's PCB board. You can wire up your electronic components based on the default pin assignments but you're also free to modify it depending on your setup. Just ensure that you're connecting MOTORX_PWM pins to a PWM enabled pin on the microcontroller and reserve SCL and SDA pins for the IMU, and pin 13 (built-in LED) for debugging.

    // INVERT ENCODER COUNTS
    #define MOTOR1_ENCODER_INV false 
    #define MOTOR2_ENCODER_INV false 
    #define MOTOR3_ENCODER_INV false 
    #define MOTOR4_ENCODER_INV false 

    // INVERT MOTOR DIRECTIONS
    #define MOTOR1_INV false
    #define MOTOR2_INV false
    #define MOTOR3_INV false
    #define MOTOR4_INV false

    // ENCODER PINS
    #define MOTOR1_ENCODER_A 14
    #define MOTOR1_ENCODER_B 15 

    #define MOTOR2_ENCODER_A 11
    #define MOTOR2_ENCODER_B 12 

    #define MOTOR3_ENCODER_A 17
    #define MOTOR3_ENCODER_B 16 

    #define MOTOR4_ENCODER_A 9
    #define MOTOR4_ENCODER_B 10

    // MOTOR PINS
    #ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
        #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can swap it with pin no 1 instead.
        #define MOTOR1_IN_A 20
        #define MOTOR1_IN_B 1 

        #define MOTOR2_PWM 5
        #define MOTOR2_IN_A 6
        #define MOTOR2_IN_B 8

        #define MOTOR3_PWM 22
        #define MOTOR3_IN_A 23
        #define MOTOR3_IN_B 0

        #define MOTOR4_PWM 4
        #define MOTOR4_IN_A 3
        #define MOTOR4_IN_B 2

        #define PWM_MAX pow(2, PWM_BITS) - 1
        #define PWM_MIN -PWM_MAX
    #endif  

Constants' Meaning:

- **MOTORX_ENCODER_A** - Microcontroller pin that is connected to the first read pin of the motor encoder. This pin is usually labelled as A pin on the motor encoder board.

- **MOTORX_ENCODER_B** - Microcontroller pin that is connected to the second read pin of the motor encoder. This pin is usually labelled as B pin on the motor encoder board.

- **MOTORX_ENCODER_INV** - Flag used to change the sign of the encoder value. More on that later.

- **MOTORX_PWM** - Microcontroller pin that is connected to the PWM pin of the motor driver. This pin is usually labelled as EN or ENABLE pin on the motor driver board. 

- **MOTORX_IN_A** - Microcontroller pin that is connected to one of the motor driver's direction pins. This pin is usually labelled as DIRA or DIR1 pin on the motor driver board. On BTS7960 driver, this is one of the two PWM pins connected to the driver (RPWM/LPWM).

- **MOTORX_IN_B** - Microcontroller pin that is connected to one of the motor driver's direction pins. This pin is usually labelled as DIRB or DIR2 pin on the motor driver board. On BTS7960 driver, this is one of the two PWM pins connected to the driver (RPWM/LPWM).

- **MOTORX_INV** - Flag used to invert the direction of the motor. More on that later.

## Calibration
Before proceeding, **ensure that your robot is elevated and the wheels aren't touching the ground**. 
5.1
### 1. Motor Check
Go to calibration folder and upload the firmware:

    cd linorobot2_hardware/calibration
    pio run --target upload -e <your_teensy_board>

Available Teensy boards:
- teensy31
- teensy35
- teensy36
- teensy40
- teensy41

Some Linux machines might encounter a problem related to libusb. If so, install libusb-dev:

    sudo apt install libusb-dev

Start spinning the motors by running:
    
    screen /dev/ttyACM0

On the terminal type `spin` and press the enter key.

The wheels will spin one by one for 10 seconds from Motor1 to Motor4. Check if each wheel's direction is spinning **forward** and take note of the motors that are spinning in the opposite direction. Set MOTORX_INV constant in [lino_base_config.h](https://github.com/linorobot/linorobot2_hardware/blob/master/config/lino_base_config.h#L71-L74) to `true` to invert the motor's direction. Reupload the calibration firmware once you're done. Press `Ctrl` + `a` + `d` to exit the screen terminal.

    cd linorobot2_hardware/calibration
    pio run --target upload -e <your_teensy_board>

### 2. Encoder Check

Open your terminal and run:

    screen /dev/ttyACM0

Type `sample` and press the enter key. Verify if all the wheels are spinning **forward**. Redo the previous step if there are motors still spinning in the opposite direction.

You'll see a summary of the total encoder readings and counts per revolution after the motors have been sampled. If you see any negative number in the MOTOR ENCODER READINGS section, invert the encoder pin by setting `MOTORX_ENCODER_INV` in [lino_base_config.h](https://github.com/linorobot/linorobot2_hardware/blob/master/config/lino_base_config.h#L65-L68) to `true`. Reupload the calibration firmware to check if the encoder pins have been reconfigured properly:

    cd linorobot2_hardware/calibration
    pio run --target upload -e <your_teensy_board>
    screen /dev/ttyACM0

Type `sample` and press the enter key. Verify if all encoder values are now **positive**. Redo this step if you missed out any.

### 3. Counts Per Revolution

On the previous instruction where you check the encoder reads for each motor, you'll see that there's also COUNTS PER REVOLUTION values printed on the screen. If you have defined `MOTOR_OPERATING_VOLTAGE` and `MOTOR_POWER_MEASURED_VOLTAGE`, you can assign these values to `COUNTS_PER_REVX` constants in [lino_base_config.h](https://github.com/linorobot/linorobot2_hardware/blob/master/config/lino_base_config.h#L55-L58) to have a more accurate model of the encoder.

## Upload the firmware
Ensure that the robot pass all the requirements before uploading the firmware:

- Defined the correct motor rpm.
- Motors' IDs are correct.
- Motors spin in the right direction.
- Encoders' signs are correct.
- Defined the correct encoder's COUNTS_PER_REV.
- Defined the correct robot type.
- Defined the correct motor driver.
- Defined the correct IMU.
- Defined the correct wheel diameter.
- Defined the correct distance between wheels.

Run:

    cd linorobot2_hardware/firmware
    pio run --target upload -e <your_teensy_board>

## Testing the robot

### 1. Run the micro-ROS agent.

This will allow the robot to receive Twist messages to control the robot, and publish odometry and IMU data straight from the microcontroller. Compared to Linorobot's ROS1 version, the odometry and IMU data published from the microcontroller use standard ROS2 messages and do not require any relay nodes to reconstruct the data to complete [sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) and [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages.

Run the agent:

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

### 2. Drive around

Run teleop_twist_keyboard package and follow the instructions on the terminal on how to drive the robot:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard 

### 3. Check the topics

Check if the odom and IMU data are published:

    ros2 topic list

Now you should see the following topics:

    /cmd_vel
    /imu/data
    /odom/unfiltered
    /parameter_events
    /rosout

Echo odometry data:

    ros2 topic echo /odom/unfiltered

Echo IMU data:

    ros2 topic echo /imu/data


## URDF
Once the hardware is done, you can go back to [linorobot2](https://github.com/NTheuws/linorobot2_software/tree/humble?tab=readme-ov-file#urdf) package and start defining the robot's URDF.

## Troubleshooting Guide

### 1. One of my motor isn't spinning.
- Check if the motors are powered.
- Check if you have bad wiring.
- Check if you have misconfigured the motor's pin assignment in `lino_base_config.h`.
- Check if you uncommented the correct motor driver (ie. `USE_GENERIC_2_IN_MOTOR_DRIVER`)
- Check if you assigned the motor driver pins under the correct motor driver constant. For instance, if you uncommented `USE_GENERIC_2_IN_MOTOR_DRIVER`, all the pins you assigned must be inside the `ifdef USE_GENERIC_2_IN_MOTOR_DRIVER` macro.

### 2. Wrong wheel is spinning during calibration process
- Check if the motor drivers have been connected to the correct microcontroller pin.
- Check if you have misconfigured the motor's pin assignment in `lino_base_config.h`.

### 3. When calibrating wheels, multiple are spinning at the same time.
-	Make sure the correct motor driver is chosen in `linorobot2_hardware/config/lino_base_config.h`.
-	Make sure the wiring is the same as the pinout in `linorobot2_hardware/config/lino_base_config.h`.
-	Make sure the wiring through the motor driver corresponds to the correct motor.
-	Make sure the PWM and DIR pins aren’t swapped.

### 4. Wheels are spinning in the wrong direction.
-	In the `linorobot2_hardware/config/lino_base_config.h` file there is a variable called `motor<x>_inv` this is used to invert the direction. X is the motor number from front left being 1 and back right being 4. If the wheel is moving the correct way, leave this be. In case it goes the other way, flip the variable of the specific motor.

### 5. The wheels only spin in one direction
- Check if the Teensy's GND pin is connected to the motor driver's GND pin.

### 6. The motor doesn't change it's direction after setting the INV to true.
- Check if the Teensy's GND pin is connected to the motor driver's GND pin.

### 7. One of my encoders has no reading (0 value).
- Check if the encoders are powered.
- Check if you have bad wiring.
- Check if you have misconfigured the encoder's pin assignment in `lino_base_config.h`.

### 8. The encoders are reading a negative value.
-	Enable/disable this value in linorobot2_hardware/config/lino_base_config.h on the right place, this is documented with comments in the file. Make sure to do this for the right motor driver used.

### 9. Some of the encoders are reading a very low value.
-	Make sure the wiring of the encoder's + and – are correctly connected.
-	They may be switched around.

### 10. Nothing's printing when I run the screen app.
- Check if you're passing the correct serial port. Run:

        ls /dev/ttyACM*
    
    and ensure that the available serial port matches the port you're passing to the screen app.

- Check if you forgot to [copy the udev rule](https://github.com/linorobot/linorobot2_hardware#3-udev-rule):

        ls /etc/udev/rules.d/00-teensy.rules 

    Remember to restart your computer if you just copied the udev rule.

### 11. The firmware was uploaded but nothing's happening.
- Check if you're assigning the correct Teensy board when uploading the firmware. If you're unsure which Teensy board you're using, take a look at the label on the biggest chip found in your Teensy board and compare it with the boards shown on PJRC's [website](https://www.pjrc.com/teensy/).

### 12. The robot's forward motion is not straight
- This happens when the target velocity is more than or equal the motor's RPM (usually happens on low RPM motors). To fix this, set the `MAX_RPM_RATIO` lower to allow the PID controller to compensate for errors.

### 13. Micro_ros_agent package isn’t recognized.
-	Make sure you’re in the right directory: linorobot2_ws.
-	Try to reinstall the following:
  ```
    source /opt/ros/<ros_distro>/setup.bash
    cd /tmp
    wget https://raw.githubusercontent.com/linorobot/linorobot2/${ROS_DISTRO}/install_linorobot2.bash
    bash install_linorobot2.bash <robot_type> <laser_sensor> <depth_sensor>
    source ~/.bashrc
```

   The agent is included in this install. In case the terminal decides to close itself follow. 

### 14. Terminal shuts down during installation of the firmware.
-	Did the raspberry run out of memory? 
    - Limit the number of parallel processes by using: `--parallel-workers` and/or `--executor sequential`.
    - Using swap memory.
    - In case you’re using a visual version of Ubuntu, disable this. `sudo systemct1 set-default multi-user` followed by a reboot. This disables it which means the Raspberry will be terminal based.

### 15. Raspberry Pi restarts itself (endless loop).
-	When starting the Raspberry, observe the red LED. If this turns off it is highly likely that the voltage is too low. The average current a Raspberry 3B should have is 2,5A so make sure this is also not the issue.

### 16. Can’t upload firmware to teensy.
-	Make sure the wire connecting the Raspberry with the teensy is a data-cable and not just a charger.
-	Make sure you’re uploading from the right directory on the Raspberry.

### 17. Error during the uploading of the firmware.
-	Press the reset button on the teensy and try again.

### 18.	Can’t run teleop_twist_keyboard since the terminal is in use.
-	Open a different terminal (`ctrl+alt+f2`) and run the command there. Switching between terminals is done with `ctrl+alt+f1` / `ctrl+alt+f2`.

### 19. Not all topics are present during topic check.
-	Make sure the micro_ros_agent is running.
-	Make sure the teleop_twist_keyboard is running.

### 20. The robot rotates after braking
- This happens due to the same reason as 7. When the motor hits its maximum rpm and fails to reach the target velocity, the PID controller's error continously increases. The abrupt turning motion is due to the PID controller's attempt to further compensate the accumulated error. To fix this, set the `MAX_RPM_RATIO` lower to allow the PID controller to compensate for errors while moving to avoid huge accumulative errors when the robot stops.

### 21. Can’t connect to the robot computer.
-	Make sure ssh is enabled on the robot computer.
    - To enable it use: `sudo systemctl enable ssh`.
        - If it isn’t installed use: `sudo apt install openssh-server`.
-	Make sure the IP of the robot user is correct.
-	Make sure the user of the robot computer is correct.

## Developers
#### Adding firmware compilation tests for a new ROS distro
To add a new distro to the CI tests, modify the `rolling` (default) branch. Inside of `.github/workflows`, duplicate an existing distro workflow YAML file. For example, to add ROS2 Iron support, one could copy `humble-firmware-build.yml` to `iron-firmware-build.yml`. Assuming that an `iron` branch exists (if not one could create one using the `humble` branch as a base and modify as necessary), inside of `iron-firmware-build.yml`, rename all instances of the word `humble` with `iron`. It would be as simple as using 'find and replace' in many IDEs. Commit these changes to a feature branch, create a PR to merge into the `rolling` branch, and then backport the PR to other branches. It is only necessary to have `iron-firmware-build.yml` on the `rolling` and `iron` branch, however it may be simpler to keep the branches in sync by having every workflow file on all branches.

Lastly, the new branch must be added to the CI table written in Markdown at the top of README.md that displays the status of each branch using badges. This table is organized with the most current ROS2 branch at the top, which is always `rolling`, and then in descending chronological order. Adding a new distro can be done by copying an existing row of the table, pasting in the appropriate position, and changing the titles and branch names in the relative paths. 
