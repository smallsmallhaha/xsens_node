#include <xcommunication/enumerateusbdevices.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/legacydatapacket.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xsportinfoarray.h>
#include <xsens/xstime.h>

#include "deviceclass.h"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>

char logger_msg_buffer[128];
const XsReal PI = 3.141592653589;
const double TIMEOUT_OF_RECV_MSG = 1.5;
ros::WallTime time_of_recv_msg;
std::string port_name;
std::string imu_frame_id;
int imu_frequency;

inline ros::WallTime getCurrentTime() { return ros::WallTime::now(); }

// publish sensor_msgs::Imu message
void publishImu(ros::Publisher &publisher, XsDataPacket &packet);
// publish nav_msgs::Odometry message
void publishOdom(ros::Publisher &publisher, XsDataPacket &packet);

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ros_node");
  ros::NodeHandle n;

  n.param<std::string>("port", port_name, "/dev/ttyUSB0");
  n.param<std::string>("imu_frame_id", imu_frame_id, "imu_link");
  n.param<int>("imu_frequency", imu_frequency, 100);

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 100);

  DeviceClass device;
  try {
    XsPortInfo mtPort(port_name, XsBaud::numericToRate(115200));

    // Open the port with the detected device
    std::cout << "Opening port..." << std::endl;
    if (!device.openPort(mtPort))
      throw std::runtime_error("Open Xsens port Failed. Aborting.");

    // Put the device in configuration mode
    std::cout << "Putting device into configuration mode..." << std::endl;
    if (!device.gotoConfig())  // Put the device into configuration mode before
                               // configuring the device
    {
      throw std::runtime_error(
          "Could not put device into configuration mode. Aborting.");
    }

    // Request the device Id to check the device type
    mtPort.setDeviceId(device.getDeviceId());

    // Check if we have an MTi / MTx / MTmk4 device
    if (!mtPort.deviceId().isMt9c() && !mtPort.deviceId().isLegacyMtig() &&
        !mtPort.deviceId().isMtMk4() && !mtPort.deviceId().isFmt_X000()) {
      throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
    }
    std::cout << "Found a device with id: "
              << mtPort.deviceId().toString().toStdString()
              << " @ port: " << mtPort.portName().toStdString()
              << ", baudrate: " << mtPort.baudrate() << std::endl;

    try {
      // Print information about detected MTi / MTx / MTmk4 device
      std::cout << "Device: " << device.getProductCode().toStdString()
                << " opened." << std::endl;

      // Configure the device. Note the differences between MTix and MTmk4
      std::cout << "Configuring the device..." << std::endl;
      if (mtPort.deviceId().isMt9c() || mtPort.deviceId().isLegacyMtig()) {
        XsOutputMode outputMode = XOM_Orientation;  // output orientation data
        XsOutputSettings outputSettings =
            XOS_OrientationMode_Quaternion;  // output orientation data as
                                             // quaternion

        // set the device configuration
        if (!device.setDeviceMode(outputMode, outputSettings)) {
          throw std::runtime_error("Could not configure MT device. Aborting.");
        }
      } else if (mtPort.deviceId().isMtMk4() ||
                 mtPort.deviceId().isFmt_X000()) {
        XsOutputConfigurationArray configArray;

        configArray.push_back(
            XsOutputConfiguration(XDI_Quaternion, imu_frequency));
        configArray.push_back(
            XsOutputConfiguration(XDI_Acceleration, imu_frequency));
        configArray.push_back(
            XsOutputConfiguration(XDI_RateOfTurn, imu_frequency));
        configArray.push_back(XsOutputConfiguration(XDI_LatLon, imu_frequency));
        configArray.push_back(
            XsOutputConfiguration(XDI_AltitudeEllipsoid, imu_frequency));

        if (!device.setOutputConfiguration(configArray)) {
          throw std::runtime_error(
              "Could not configure MTmk4 device. Aborting.");
        }
      } else {
        throw std::runtime_error("Unknown device while configuring. Aborting.");
      }

      // Put the device in measurement mode
      std::cout << "Putting device into measurement mode..." << std::endl;
      if (!device.gotoMeasurement()) {
        throw std::runtime_error(
            "Could not put device into measurement mode. Aborting.");
      }

      std::cout << "Measuring..." << std::endl;

      XsByteArray data;
      XsMessageArray msgs;
      // initialize time_of_recv_msg in case of program exits at first loop
      time_of_recv_msg = getCurrentTime();
      while (ros::ok()) {
        device.readDataToBuffer(data);
        device.processBufferedData(data, msgs);
        if (msgs.empty()) {
          // if new message not arrive in TIMEOUT_OF_RECV_MSG seconds, exit the
          // program
          if ((getCurrentTime() - time_of_recv_msg).toSec() >
              TIMEOUT_OF_RECV_MSG) {
            throw std::runtime_error("USB DEVICE CLOSED.");
          }
        } else {
          time_of_recv_msg = getCurrentTime();
        }
        for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end();
             ++it) {
          // Retrieve a packet
          XsDataPacket packet;
          if ((*it).getMessageId() == XMID_MtData) {
            LegacyDataPacket lpacket(1, false);
            lpacket.setMessage((*it));
            lpacket.setXbusSystem(false);
            lpacket.setDeviceId(mtPort.deviceId(), 0);
            lpacket.setDataFormat(XOM_Orientation,
                                  XOS_OrientationMode_Quaternion,
                                  0);  // lint !e534
            XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
          } else if ((*it).getMessageId() == XMID_MtData2) {
            packet.setMessage((*it));
            packet.setDeviceId(mtPort.deviceId());
          }

          if (packet.containsCalibratedGyroscopeData() &&
              packet.containsCalibratedAcceleration()) {
            if (packet.containsPositionLLA()) {
              publishOdom(odom_pub, packet);
            } else {
              publishImu(imu_pub, packet);
            }
          } else {
            throw std::runtime_error("gyro or accelarator data not found");
          }
        }
        msgs.clear();
        XsTime::msleep(0);
      }
    } catch (std::runtime_error const &error) {
      sprintf(logger_msg_buffer, "XSENS: %s", error.what());
      ROS_ERROR_STREAM(logger_msg_buffer);
    } catch (...) {
      ROS_ERROR_STREAM("XSENS: An unknown fatal error has occured. Aborting.");
    }

    // Close port
    std::cout << "Closing port..." << std::endl;
    device.close();
  } catch (std::runtime_error const &error) {
    sprintf(logger_msg_buffer, "XSENS: %s", error.what());
    ROS_ERROR_STREAM(logger_msg_buffer);
  } catch (...) {
    ROS_ERROR_STREAM("XSENS: An unknown fatal error has occured. Aborting.");
  }

  std::cout << "EXIT" << std::endl;

  return 0;
}

void publishImu(ros::Publisher &publisher, XsDataPacket &packet) {
  auto quat = packet.orientationQuaternion();
  auto gyro = packet.calibratedGyroscopeData();
  auto acce = packet.calibratedAcceleration();

  sensor_msgs::Imu imu;
  imu.header.stamp.fromNSec(time_of_recv_msg.toNSec());
  imu.header.frame_id = imu_frame_id;

  // set orientation
  imu.orientation.x = quat.x();
  imu.orientation.y = quat.y();
  imu.orientation.z = quat.z();
  imu.orientation.w = quat.w();

  // set angular velocity (rad/s)
  imu.angular_velocity.x = gyro[0];
  imu.angular_velocity.y = gyro[1];
  imu.angular_velocity.z = gyro[2];

  // set linear acceleration (m/s2)
  imu.linear_acceleration.x = acce[0];
  imu.linear_acceleration.y = acce[1];
  imu.linear_acceleration.z = acce[2];

  // publish the message
  publisher.publish(imu);
}

void BLH2XYZ(XsReal *BLH, double *XYZ) {
  double B = BLH[0] * PI / 180;
  double L = BLH[1] * PI / 180;
  double H = BLH[2];
  double a = 6378137;
  double e2 = 0.00669437999013;

  double N = a / sqrt(1 - e2 * sin(B) * sin(B));
  XYZ[0] = (N + H) * cos(B) * cos(L);
  XYZ[1] = (N + H) * cos(B) * sin(L);
  XYZ[2] = (N * (1 - e2) + H) * sin(B);
}

void publishOdom(ros::Publisher &publisher, XsDataPacket &packet) {
  if (!packet.containsPositionLLA()) return;

  auto quat = packet.orientationQuaternion();
  auto pLLA = packet.positionLLA();
  auto gyro = packet.calibratedGyroscopeData();
  auto acce = packet.calibratedAcceleration();

  nav_msgs::Odometry odom;

  odom.child_frame_id = "base_link";
  odom.header.stamp.fromNSec(time_of_recv_msg.toNSec());
  odom.header.frame_id = "odom";

  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();

  double XYZ[3];
  BLH2XYZ(pLLA.toVector().data(), XYZ);
  odom.pose.pose.position.x = XYZ[0];
  odom.pose.pose.position.y = XYZ[1];
  odom.pose.pose.position.z = XYZ[2];

  odom.twist.twist.angular.x = gyro[0];
  odom.twist.twist.angular.y = gyro[1];
  odom.twist.twist.angular.z = gyro[2];

  odom.twist.twist.linear.x = acce[0];
  odom.twist.twist.linear.y = acce[1];
  odom.twist.twist.linear.z = acce[2];

  publisher.publish(odom);
}
