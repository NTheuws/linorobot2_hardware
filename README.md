# Installation

## 1. Installing ROS2 and micro-ROS in the host computer

## 1.1 ROS2 Installation

You can use the script found in [ros2me](https://github.com/linorobot/ros2me) to install ROS2 - Foxy. Take note that this project only works on ROS Foxy and above.

    git clone https://github.com/linorobot/ros2me
    cd ros2me
    ./install

You'll also need the teleoperation package to control the robot manually. Install ROS2's teleop_twist_keyboard package:

    sudo apt install ros-foxy-teleop-twist-keyboard 

## 1.2 micro-ROS Installation

Source your ROS2 distro and workspace:

    source /opt/ros/<your_ros_distro>/setup.bash
    cd <your_ws>
    colcon build
    source install/local_setup.bash

Download and install micro-ROS:

    cd <your_ws>
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    sudo apt install python3-vcstool
    sudo apt update && rosdep update
    rosdep install --from-path src --ignore-src -y
    colcon build
    source install/local_setup.bash

Setup micro-ROS agent:

    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash

## 2. Install PlatformIO
Download and install platformio:
    
    python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"

Add platformio to your $PATH:

    echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> ~/.bashrc


## 3. UDEV Rule
Download the udev rules from Teensy's website:

    wget https://www.pjrc.com/teensy/00-teensy.rules

and copy the file to /etc/udev/rules.d :

    sudo cp 00-teensy.rules /etc/udev/rules.d/

## 4. Configure robot settings

Go to config folder and open lino_base_config.h. Uncomment the base, motor driver and IMU you want to use for your robot. For example:

    #define LINO_BASE DIFFERENTIAL_DRIVE
    #define USE_GENERIC_2_IN_MOTOR_DRIVER
    #define USE_GY85_IMU

Constants Meaning:

*ROBOT TYPE (LINO_BASE)*
- **DIFFERENTIAL_DRIVE** - 2 wheeled drive or tracked robots w/ 2 motors.

- **SKID_STEER** - 4 wheeled drive robots.

- **MECANUM** - 4 wheeled drive robots using mecanum wheels.

*MOTOR DRIVERS*
- **USE_GENERIC_2_IN_MOTOR_DRIVER** - Motor drivers that has EN (pwm) pin, and 2 direction pins (usually DIRA, DIRB pins).

- **USE_GENERIC_1_IN_MOTOR_DRIVER** - Motor drivers that has EN (pwm) pin, and 1 direction pin (usual DIR pin). These drivers usually have logic gates included to lessen the pins required in controlling the driver.

- **USE_BTS7960_MOTOR_DRIVER** - BTS7960 motor driver.

- **USE_ESC_MOTOR_DRIVER** - Bi-directional (forward/reverse) electronic speed controllers.

*INERTIAL MEASUREMENT UNIT (IMU)*
- **USE_GY85_IMU** - GY-85 IMUs.

- **USE_MPU6050_IMU** - MPU6060 IMUs.

- **USE_MPU9150_IMU** - MPU9150 IMUs.

- **USE_MPU9250_IMU** - MPU9250 IMUs.

Next, fill in the robot settings accordingly:

    #define K_P 0.6 // P constant
    #define K_I 0.8 // I constant
    #define K_D 0.5 // D constant

    //define your robot' specs here
    #define MOTOR_MAX_RPM 100             
    #define MOTOR_OPERATING_VOLTAGE 12
    #define MOTOR_POWER_MEASURED_VOLTAGE 11.7

    #define COUNTS_PER_REV1 2200    
    #define COUNTS_PER_REV2 2200      
    #define COUNTS_PER_REV3 2200      
    #define COUNTS_PER_REV4 2200      
  
    #define WHEEL_DIAMETER 0.09  

    #define LR_WHEELS_DISTANCE 0.2  

    #define FR_WHEELS_DISTANCE 0.30  

    #define PWM_BITS 8

Constants Meaning:

- **K_P, K_I, K_D** - PID constants used to translate the robot's target velocity to motor speed.

- **MOTOR_MAX_RPM** - Maximum number of rotation your motor can do in a minute specified by the manufacturer.

- **MOTOR_OPERATING_VOLTAGE** - Operating voltage of the motor specified by the manufacturer. This parameter is used to calculate the motor encoder's `COUNTS_PER_REV` constant. You can ignore this if you're using the manufacturer's specified counts per rev.

- **MOTOR_POWER_MEASURED_VOLTAGE** - Measured voltage of the motor's power source. This parameter is used to calculate the motor encoder's `COUNTS_PER_REV` constant. You can ignore this if you're using the manufacturer's specified counts per rev.

- **COUNTS_PER_REVX** - The total number of pulses the encoder has to read to be considered as one revolution. You can either use the manufacturer's specification or the calibrated value in the next step. If you're planning to use the calibrated value, ensurethat you have defined the correct values for `MOTOR_OPERATING_VOLTAGE` and `MOTOR_POWER_MEASURED_VOLTAGE`.

- **WHEEL_DIAMETER** - Diameter of the wheels.

- **LR_WHEELS_DISTANCE** - The distance between the center of left and right wheels in meters.

- **FR_WHEELS_DISTANCE** - The distance between the center of front and rear wheels in meters. This only applies to 4wd and mecanum robots. Just use the default value if you're using a 2wd robot.

- **PWM_BITS** - Number of bits in generating the PWM signal. You can leave this value as is unless you have very special use case.

and the microcontroller pins connected to the motors and encoders. Remember to only modify the correct constants under the motor controller macro that you're using ie. USE_GENERIC_2_IN_MOTOR_DRIVER. 

Robot Orientation:

--------------FRONT--------------

WHEEL1 WHEEL2 (2WD)

WHEEL3 WHEEL4 (4WD)

--------------BACK--------------

    #define MOTOR1_ENCODER_A 14
    #define MOTOR1_ENCODER_B 15 
    #define MOTOR1_ENCODER_INV false 

    #define MOTOR2_ENCODER_A 11
    #define MOTOR2_ENCODER_B 12 
    #define MOTOR2_ENCODER_INV false 

    #define MOTOR3_ENCODER_A 17
    #define MOTOR3_ENCODER_B 16 
    #define MOTOR3_ENCODER_INV false 

    #define MOTOR4_ENCODER_A 9
    #define MOTOR4_ENCODER_B 10
    #define MOTOR4_ENCODER_INV false 

    #ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
    #define MOTOR1_PWM 21
    #define MOTOR1_IN_A 20
    #define MOTOR1_IN_B 1
    #define MOTOR1_INV false

    #define MOTOR2_PWM 5
    #define MOTOR2_IN_A 6
    #define MOTOR2_IN_B 8
    #define MOTOR2_INV false

    #define MOTOR3_PWM 22
    #define MOTOR3_IN_A 23
    #define MOTOR3_IN_B 0
    #define MOTOR3_INV false

    #define MOTOR4_PWM 4
    #define MOTOR4_IN_A 3
    #define MOTOR4_IN_B 2
    #define MOTOR4_INV false

    #define PWM_MAX pow(2, PWM_BITS) - 1
    #define PWM_MIN -PWM_MAX
    #endif 

Constants Meaning:
- **MOTORX_ENCODER_A** - First read pin of the motor encoder. This pin is usually labelled as A pin.

- **MOTORX_ENCODER_B** - Second read pin of the motor encoder. This pin is usually labelled as B pin.

- **MOTORX_ENCODER_INV** - Flag used to change the sign of the encoder value. More on that later.

- **MOTORX_PWM** - Pin used to control the speed of the motor. This pin is usually labelled as EN or ENABLE pin.

- **MOTORX_IN_A** - Pin used to control the direction of the motor.This pin is usually labelled as DIRA or DIR1 pin.

- **MOTORX_IN_B** - Pin used to control the direction of the motor. This pin is usually labelled as DIRB or DIR2 pin.

- **MOTORX_INV** - Flag used to invert the direction of the motor. More on that later.

## 5. Motor and Encoder Checks

### 5.1 Motor Check
Before proceeding, **ensure that your robot is elavated and the wheels aren't touching the ground**. 

Go to calibration folder and upload the firmware. The robot will start spinning once the firmware has been uploaded, so **ensure that nothing is obstructing the wheels**.

    cd linorobot2_prototype/calibration
    pio run --target upload -e <your_teensy_board>


Available Teensy boards:
- teensy31
- teensy35
- teensy36
- teensy40
- teensy41

Some Linux machines might encounter a problem related to libusb. If so, install libusb-dev:

    sudo apt install libusb-dev

Once the wheels start spinning, check if each wheel's direction is spinning forward. Take note of the motors that are spinning in the opposite direction and set the MOTORX_INV constant in lino_base_config.h to `true` to invert the motor's direction. Reupload the calibration firmware to check if the wheels have been reconfigured properly:

    cd linorobot2_prototype/calibration
    pio run --target upload -e <your_teensy_board>

Verify if all the wheels are spinning forward.

### 5.2 Encoder Check

Open your terminal and run:

    screen /dev/ttyACM0 115200

Wait for the logs to print and look for the values of M1, M2, M3 and M4. If you see any negative number in one of the motors, set `MOTORX_ENCODER_INV` in lino_base_config.h to `true` to invert the encoder pin. Reupload the calibration firmware to check if the encoder pins have been reconfigured properly:

    cd linorobot2_prototype/calibration
    pio run --target upload -e <your_teensy_board>
    screen /dev/ttyACM0 115200

Verify if all encoder values are now positive.

### 5.3 Counts Per Revolution

On the previous instruction where you check the encoder reads for each motor, you'll see that there's also CPR values printed on the screen. If you have defined `MOTOR_OPERATING_VOLTAGE` and `MOTOR_POWER_MEASURED_VOLTAGE`, you can assign these values to `COUNTS_PER_REVX` constants in lino_base_config.h to have a more accurate model of the encoder.

### 5.4 Troubleshooting Guide

#### 5.4.1 One of my motor's not spinning.
- Check if the motors are powered.
- Check if you have a bad wiring.
- Check if you have misconfigured the motor's pin assignment in lino_base_config.h.
- Check if you uncommented the correct motor driver (ie. `USE_GENERIC_2_IN_MOTOR_DRIVER`)
- Check if you assigned the motor driver pins under the correct motor driver constant. For instance, if you uncommented `USE_GENERIC_2_IN_MOTOR_DRIVER`, all the pins you assigned must be inside the `ifdef USE_GENERIC_2_IN_MOTOR_DRIVER` macro.

#### 5.4.2 One of my encoder has no reading (0 value).
- Check if the encoders are powered.
- Check if you have a bad wiring.
- Check if you have misconfigured the encoder's pin assignment in lino_base_config.h.

#### 5.4.3 Nothing's printing when I run the screen app.
- Check if you're passing the correct serial port. Run:

        ls /dev/ttyACM*
    
    and ensure that the available serial port matches the port you're passing to the screen app.

- Check if you forgot to [copy the udev rule](https://github.com/linorobot/linorobot2_prototype#3-udev-rule). Remember to restart your computer if you just copied the udev rule.

#### 5.4.4 The firmware uploaded but the nothing's happening.
- Check if you're assigning the correct Teensy board when uploading the firmware. If you're unsure which Teensy board you're using, take a look at the label on the biggest chip found in your Teensy board and compare it with the boards shown on PJRC's [website](https://www.pjrc.com/teensy/).


## 6. Upload the firmware

Upload the firmware by running:

    cd linorobot2_prototype/firmware
    pio run --target upload -e <your_teensy_board>

# Running the demo

## 1. Run the micro-ROS agent.

This will allow the robot to receive Twist messages to control the robot, and publish odometry and imu data straight from the microcontroller. Compared to Linorobot's ROS1 version, the odometry and IMU data published from the microcontroller uses standard ROS2 messages and doesn't require any relay nodes to reconstruct the data to full fledge [sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) and [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages.

Run the agent:

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

## 2. Drive around

Run teleop_twist_keyboard package and follow the instructions on the terminal on how to drive the robot:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard 

## 3. Check the topics

Check if the odom and IMU data are published:

    ros2 topic list

Now you should see the following topics:

    /cmd_vel
    /imu/data
    /odom
    /parameter_events
    /rosout

You can subscribe to any of the topics by running:

    ros2 topic echo odom