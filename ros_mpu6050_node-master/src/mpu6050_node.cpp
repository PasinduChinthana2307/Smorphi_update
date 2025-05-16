#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Typically, motion processing algorithms should be run at a high rate, often around 200Hz,
//in order to provide accurate results with low latency. This is required even if the application
//updates at a much lower rate; for example, a low power user interface may update as slowly
//as 5Hz, but the motion processing should still run at 200Hz.
//Page 25 of MPU6050 datasheet.
#define DEFAULT_SAMPLE_RATE_HZ	10

#define MPU_FRAMEID "base_imu"

// MPU6050 mpu instance
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// MPU Configuration 
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

// ROS2 Node Class for MPU6050
class MPU6050Node : public rclcpp::Node
{
public:
  MPU6050Node() : Node("mpu6050_node")
  {
    // Declare parameters
    this->declare_parameter("frequency", DEFAULT_SAMPLE_RATE_HZ);
    this->declare_parameter("frame_id", MPU_FRAMEID);
    this->declare_parameter("ax", 0);
    this->declare_parameter("ay", 0);
    this->declare_parameter("az", 0);
    this->declare_parameter("gx", 0);
    this->declare_parameter("gy", 0);
    this->declare_parameter("gz", 0);
    this->declare_parameter("ado", false);
    this->declare_parameter("debug", false);
    this->declare_parameter("linear_acceleration_stdev", (400 / 1000000.0) * 9.807);
    this->declare_parameter("angular_velocity_stdev", 0.05 * (M_PI / 180.0));
    this->declare_parameter("pitch_roll_stdev", 1.0 * (M_PI / 180.0));
    this->declare_parameter("yaw_stdev", 5.0 * (M_PI / 180.0));

    // Get parameters
    sample_rate = this->get_parameter("frequency").as_int();
    frame_id = this->get_parameter("frame_id").as_string();
    ax = this->get_parameter("ax").as_int();
    ay = this->get_parameter("ay").as_int();
    az = this->get_parameter("az").as_int();
    gx = this->get_parameter("gx").as_int();
    gy = this->get_parameter("gy").as_int();
    gz = this->get_parameter("gz").as_int();
    ado = this->get_parameter("ado").as_bool();
    debug = this->get_parameter("debug").as_bool();
    
    double linear_acceleration_stdev_ = this->get_parameter("linear_acceleration_stdev").as_double();
    double angular_velocity_stdev_ = this->get_parameter("angular_velocity_stdev").as_double();
    double pitch_roll_stdev_ = this->get_parameter("pitch_roll_stdev").as_double();
    double yaw_stdev_ = this->get_parameter("yaw_stdev").as_double();

    angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_;
    yaw_covariance = yaw_stdev_ * yaw_stdev_;

    RCLCPP_INFO(this->get_logger(), "Starting mpu6050_node...");
    RCLCPP_INFO(this->get_logger(), "Using sample rate: %d", sample_rate);
    RCLCPP_INFO(this->get_logger(), "Using frame_id: %s", frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "ADO: %s", ado ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Debug: %s", debug ? "true" : "false");

    // Create publishers
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    imu_euler_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/euler", 10);
    mag_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/mag", 10);

    // Initialize MPU6050
    initializeMPU();

    // Create timer for main loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / sample_rate),
      std::bind(&MPU6050Node::loop, this));
  }

private:
  void initializeMPU()
  {
    RCLCPP_INFO(this->get_logger(), "Initializing I2C...");
    I2Cdev::initialize();

    // Verify connection
    RCLCPP_INFO(this->get_logger(), "Testing device connections...");
    mpu = MPU6050(ado ? 0x69 : 0x68);
    if(mpu.testConnection()) {
      RCLCPP_INFO(this->get_logger(), "MPU6050 connection successful");
    } else {
      RCLCPP_ERROR(this->get_logger(), "MPU6050 connection failed");
      rclcpp::shutdown();
      exit(1);
    }

    // Initialize device
    RCLCPP_INFO(this->get_logger(), "Initializing I2C devices...");
    mpu.initialize();

    // Load and configure the DMP
    RCLCPP_INFO(this->get_logger(), "Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // Set accel offsets
    RCLCPP_INFO(this->get_logger(), "Setting X accel offset: %d", ax);
    mpu.setXAccelOffset(ax);
    RCLCPP_INFO(this->get_logger(), "Setting Y accel offset: %d", ay);
    mpu.setYAccelOffset(ay);
    RCLCPP_INFO(this->get_logger(), "Setting Z accel offset: %d", az);
    mpu.setZAccelOffset(az);

    // Set gyro offsets
    RCLCPP_INFO(this->get_logger(), "Setting X gyro offset: %d", gx);
    mpu.setXGyroOffset(gx);
    RCLCPP_INFO(this->get_logger(), "Setting Y gyro offset: %d", gy);
    mpu.setYGyroOffset(gy);
    RCLCPP_INFO(this->get_logger(), "Setting Z gyro offset: %d", gz);
    mpu.setZGyroOffset(gz);

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // Turn on the DMP, now that it's ready
      RCLCPP_INFO(this->get_logger(), "Enabling DMP...");
      mpu.setDMPEnabled(true);

      mpuIntStatus = mpu.getIntStatus();

      // Set our DMP Ready flag so the main loop() function knows it's okay to use it
      RCLCPP_INFO(this->get_logger(), "DMP ready!");
      dmpReady = true;

      // Get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      RCLCPP_ERROR(this->get_logger(), "DMP Initialization failed (code %d)", devStatus);
    }

    usleep(100000);
  }

  void loop()
  {
    // If programming failed, don't try to do anything
    if (!dmpReady) return;

    // Get current time for message timestamps
    auto now = this->now();

    // Create messages
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = now;
    imu_msg->header.frame_id = frame_id;

    auto imu_euler_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
    imu_euler_msg->header.stamp = now;
    imu_euler_msg->header.frame_id = frame_id;

    auto mag_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
    mag_msg->header.stamp = now;
    mag_msg->header.frame_id = frame_id;

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
      // Reset so we can continue cleanly
      mpu.resetFIFO();
      if(debug) RCLCPP_WARN(this->get_logger(), "FIFO overflow!");
    } else if (fifoCount >= 42) {
      // Read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // Display quaternion values
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      if(debug) RCLCPP_INFO(this->get_logger(), "quat %7.2f %7.2f %7.2f %7.2f", q.w, q.x, q.y, q.z);

      imu_msg->orientation.x = q.x;
      imu_msg->orientation.y = q.y;
      imu_msg->orientation.z = q.z;
      imu_msg->orientation.w = q.w;

      // Set covariance matrices
      imu_msg->linear_acceleration_covariance[0] = linear_acceleration_covariance;
      imu_msg->linear_acceleration_covariance[4] = linear_acceleration_covariance;
      imu_msg->linear_acceleration_covariance[8] = linear_acceleration_covariance;

      imu_msg->angular_velocity_covariance[0] = angular_velocity_covariance;
      imu_msg->angular_velocity_covariance[4] = angular_velocity_covariance;
      imu_msg->angular_velocity_covariance[8] = angular_velocity_covariance;

      imu_msg->orientation_covariance[0] = pitch_roll_covariance;
      imu_msg->orientation_covariance[4] = pitch_roll_covariance;
      imu_msg->orientation_covariance[8] = yaw_covariance;

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // Display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Should be in rad/sec
        imu_msg->angular_velocity.x = ypr[2];
        imu_msg->angular_velocity.y = ypr[1];
        imu_msg->angular_velocity.z = ypr[0];

        if(debug) RCLCPP_INFO(this->get_logger(), "ypr (degrees) %7.2f %7.2f %7.2f",
                              ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
        // Display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        // Convert to m/s^2
        imu_msg->linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
        imu_msg->linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
        imu_msg->linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;

        if(debug) RCLCPP_INFO(this->get_logger(), "areal (raw) %6d %6d %6d",
                             aaReal.x, aaReal.y, aaReal.z);
        if(debug) RCLCPP_INFO(this->get_logger(), "areal (m/s^2) %f %f %f",
                             imu_msg->linear_acceleration.x,
                             imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);
      #endif

      // Publish IMU data
      imu_pub->publish(std::move(imu_msg));
    }
  }

  // ROS Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_euler_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag_pub;
  
  // Timer for main loop
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  int sample_rate;
  std::string frame_id;
  int ax, ay, az, gx, gy, gz;
  bool ado;
  bool debug;
  
  // Covariance parameters
  double angular_velocity_covariance, pitch_roll_covariance;
  double yaw_covariance, linear_acceleration_covariance;
};

int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  // Create and spin the node
  auto node = std::make_shared<MPU6050Node>();
  rclcpp::spin(node);
  
  // Clean up
  rclcpp::shutdown();
  return 0;
}
