// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "lino_base_config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"

#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t publish_timer;
rcl_timer_t control_timer;

unsigned long prev_cmd_time = 0;
bool micro_ros_init_successful = false;
bool new_command = false;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV, MOTOR4_ENCODER_INV);

Motor motor1_controller(MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);
Odometry odometry;
IMU imu;

void publishCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
        odom_msg = odometry.getData();
        imu_msg = imu.getData();

        RCSOFTCHECK(rcl_publish( &imu_publisher, &imu_msg, NULL));
        RCSOFTCHECK(rcl_publish( &odom_publisher, &odom_msg, NULL));
    }
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
        // brake if there's no command received, or when it's only the first command sent
        // first command is ignored if it's less than 5hz to prevent jerky motion. ie, there's a long pause after 
        // the key is pressed in teleop_twist_keyboard
        if(((millis() - prev_cmd_time) >= 200) || new_command) 
        {
            new_command = false;
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.0;

            digitalWrite(LED_PIN, HIGH);
        }
        // get the required rpm for each motor based on required velocities, and base used
        Kinematics::rpm req_rpm = kinematics.getRPM(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);

        // get the current speed of each motor
        int current_rpm1 = motor1_encoder.getRPM();
        int current_rpm2 = motor2_encoder.getRPM();
        int current_rpm3 = motor3_encoder.getRPM();
        int current_rpm4 = motor4_encoder.getRPM();

        // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
        // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
        motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
        motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
        motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
        motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

        Kinematics::velocities current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);

        odometry.update(current_vel.linear_x, current_vel.linear_y, current_vel.angular_z);
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    // detect first command (ie. when user just started pressing accelerator)
    unsigned long now = millis();
    if((now - prev_cmd_time) > 200)
    {
        new_command = true;
    }
    prev_cmd_time = millis();
}

void createEntities()
{
    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init( &support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default( &node, "linorobot_base_node", "", &support));

    // create odometry publisher
    RCCHECK(rclc_publisher_init_best_effort( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"))

    // create IMU publisher
    RCCHECK(rclc_publisher_init_best_effort( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_best_effort( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // create timer for publishing data at 20 Hz (1000/50)
    const unsigned int publish_timeout = 50;
    RCCHECK(rclc_timer_init_default( 
        &publish_timer, 
        &support,
        RCL_MS_TO_NS(publish_timeout),
        publishCallback));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));

    // create executor
    RCCHECK(rclc_executor_init( &executor, &support.context, 3, & allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_subscription( &executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA));

    digitalWrite(LED_PIN, HIGH);
    micro_ros_init_successful = true;
}

void destroyEntities()
{
    digitalWrite(LED_PIN, LOW);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);

    rcl_node_fini(&node);
    rcl_timer_fini(&publish_timer);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    micro_ros_init_successful = false;
}

void setup() 
{
    pinMode(LED_PIN, OUTPUT);
    delay(1000);

    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        while(1)
        {
            flashLED(3);
        }
    }
    micro_ros_init_successful = false;

    set_microros_transports();
}

void loop() 
{
    static unsigned long prev_connect_test_time;

    // check if the agent got disconnected at 10Hz
    if(micros() - prev_connect_test_time > 100000)
    {
        prev_connect_test_time = micros();
        // check if the agent is connected
        if(RMW_RET_OK == rmw_uros_ping_agent(10, 2))
        {
            // reconnect if agent got disconnected or haven't at all
            if (!micro_ros_init_successful) 
            {
                createEntities();
            } 
        } 
        else if(micro_ros_init_successful)
        {
            // stop the robot when the agent got disconnected
            fullStop();
            // clean up micro-ROS components
            destroyEntities();
        }
    }
   
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
}

void rclErrorLoop() 
{
    while(true)
    {
        flashLED(2);
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

void fullStop()
{
    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}
