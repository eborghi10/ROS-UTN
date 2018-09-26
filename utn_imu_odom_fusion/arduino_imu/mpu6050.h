#pragma once

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <ros.h>
#include <rosabridge_msgs/Imu.h>

class IMUSensor
{
private:
    void begin();
    static void dmpDataReady();
    void isr();
    static IMUSensor* imuSensor;
    MPU6050 mpu;
    // Constants
    static const uint8_t INT_PIN;
    static const char* TOPIC_NAME;
    static const char* IMU_TF_LINK;
    static const double GRAVITY;
    // Interrupt pin
    uint8_t interruptPin;
    // indicates whether MPU interrupt pin has gone high
    volatile bool mpuInterrupt;

    // MPU control/status vars
    bool dmpReady;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    // ROS
    void generateIMUMessage();
    void publishIMU();
    ros::NodeHandle nh;
    rosabridge_msgs::Imu msg;
    ros::Publisher pub;
    const char base_imu_link[];
public:
    // Constructors
    IMUSensor();
    IMUSensor(uint8_t);
    IMUSensor(const char*);
    IMUSensor(uint8_t, const char*);
    // Public method
    void publish();
};

IMUSensor::IMUSensor()
: IMUSensor(INT_PIN, TOPIC_NAME) {}

IMUSensor::IMUSensor(uint8_t interruptPin)
: IMUSensor(interruptPin, TOPIC_NAME) {}

IMUSensor::IMUSensor(const char* topicName)
: IMUSensor(INT_PIN, topicName) {}

IMUSensor::IMUSensor(uint8_t interruptPin, const char* topicName)
: interruptPin(interruptPin)
, mpuInterrupt(false)
, dmpReady(false)
, pub(topicName, &msg)
, base_imu_link {"base_imu_link"}
{
    begin();
    imuSensor = this;
    nh.initNode();
    nh.advertise(pub);
    generateIMUMessage();
}

/**
 * INTERRUPT DETECTION ROUTINE
 */
void IMUSensor::dmpDataReady() {
    imuSensor->isr();
}
void IMUSensor::isr() {
    mpuInterrupt = true;
}

void IMUSensor::generateIMUMessage() {
    msg.header.seq = 0;
    msg.header.frame_id = base_imu_link;
    msg.header.frame_id = IMU_TF_LINK;

    // CBA Set IMU covariances
    msg.orientation_covariance[0] = 0.0025;
    msg.orientation_covariance[1] = 0.0025;
    msg.orientation_covariance[2] = 0.0025;
    msg.angular_velocity_covariance[0] = 0.02;
    msg.angular_velocity_covariance[1] = 0.02;
    msg.angular_velocity_covariance[2] = 0.02;
    msg.linear_acceleration_covariance[0] = 0.04;
    msg.linear_acceleration_covariance[1] = 0.04;
    msg.linear_acceleration_covariance[2] = 0.04;
}

void IMUSensor::publishIMU() {
    msg.header.seq++;
    msg.header.stamp = nh.now();

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    msg.orientation.x = q.x;
    msg.orientation.y = q.y;
    msg.orientation.z = q.z;
    msg.orientation.w = q.w;
    msg.angular_velocity.x = ypr[0];
    msg.angular_velocity.y = ypr[1];
    msg.angular_velocity.z = ypr[2];
    msg.linear_acceleration.x = aaReal.x * GRAVITY / 16384;
    msg.linear_acceleration.y = aaReal.y * GRAVITY / 16384;
    msg.linear_acceleration.z = aaReal.z * GRAVITY / 16384;

    pub.publish( &msg );
}

void IMUSensor::begin() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    // nh.loginfo("Initializing I2C devices...");
    mpu.initialize();
    pinMode(interruptPin, INPUT);

    // verify connection
    // nh.loginfo("Testing device connections...");
    // nh.loginfo(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // load and configure the DMP
    // nh.loginfo("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-1169);
    mpu.setYAccelOffset(744);
    mpu.setZAccelOffset(1620);
    mpu.setXGyroOffset(48);
    mpu.setYGyroOffset(47);
    mpu.setZGyroOffset(-8);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        // nh.loginfo("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // nh.loginfo("Enabling interrupt detection (Arduino external interrupt 0)...");
        attachInterrupt(digitalPinToInterrupt(interruptPin), IMUSensor::dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // nh.loginfo("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        char *error_msg;
        sprintf(error_msg, "DMP Initialization failed (code %d)", devStatus);
        // nh.logerror(error_msg);
    }
}

void IMUSensor::publish() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // nh.logwarn("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        publishIMU();
    }
    // Update ROS
    nh.spinOnce();
}

IMUSensor* IMUSensor::imuSensor = nullptr;
const uint8_t IMUSensor::INT_PIN = 2;
const char* IMUSensor::TOPIC_NAME = "rosabridge/imu";
const char* IMUSensor::IMU_TF_LINK = "imu_link";
const double IMUSensor::GRAVITY = 9.80665;
