/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "deviceclass.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>


#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>


char logger_msg_buffer[128];
const XsReal PI=3.141592653589;

// publish sensor_msgs::Imu message
void publishImu(ros::Publisher &publisher,XsDataPacket &packet);
// publish nav_msgs::Odometry message
void publishOdom(ros::Publisher &publisher, XsDataPacket &packet);


int main(int argc, char* argv[])
{
	DeviceClass device;
	
	ros::init(argc, argv, "imuPAGS");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry",100);
	
	std::string portName;;
	n.param<std::string>("port", portName, "/dev/ttyUSB0");

	try
	{
		XsPortInfo mtPort(portName, XsBaud::numericToRate(115200));

		// Open the port with the detected device
		std::cout << "Opening port..." << std::endl;
		if (!device.openPort(mtPort))
			throw std::runtime_error("Open Xsens port Failed. Aborting.");

		// Put the device in configuration mode
		std::cout << "Putting device into configuration mode..." << std::endl;
		if (!device.gotoConfig()) // Put the device into configuration mode before configuring the device
		{
			throw std::runtime_error("Could not put device into configuration mode. Aborting.");
		}

		// Request the device Id to check the device type
		mtPort.setDeviceId(device.getDeviceId());

		// Check if we have an MTi / MTx / MTmk4 device
		if (!mtPort.deviceId().isMt9c() && !mtPort.deviceId().isLegacyMtig() && !mtPort.deviceId().isMtMk4() && !mtPort.deviceId().isFmt_X000())
		{
			throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
		}
		std::cout << "Found a device with id: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

		try
		{
			// Print information about detected MTi / MTx / MTmk4 device
			std::cout << "Device: " << device.getProductCode().toStdString() << " opened." << std::endl;

			// Configure the device. Note the differences between MTix and MTmk4
			std::cout << "Configuring the device..." << std::endl;
			if (mtPort.deviceId().isMt9c() || mtPort.deviceId().isLegacyMtig())
			{
				XsOutputMode outputMode = XOM_Orientation; // output orientation data
				XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

				// set the device configuration
				if (!device.setDeviceMode(outputMode, outputSettings))
				{
					throw std::runtime_error("Could not configure MT device. Aborting.");
				}
			}
			else if (mtPort.deviceId().isMtMk4() || mtPort.deviceId().isFmt_X000())
			{
				XsOutputConfigurationArray configArray;
				
				configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));				
				configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
				configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
				configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
				configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
				
				if (!device.setOutputConfiguration(configArray))
				{
					throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
				}
			}
			else
			{
				throw std::runtime_error("Unknown device while configuring. Aborting.");
			}

			// Put the device in measurement mode
			std::cout << "Putting device into measurement mode..." << std::endl;
			if (!device.gotoMeasurement())
			{
				throw std::runtime_error("Could not put device into measurement mode. Aborting.");
			}

			std::cout << "Measuring..." << std::endl;

			XsByteArray data;
			XsMessageArray msgs;
			while (ros::ok())
			{
				device.readDataToBuffer(data);
				device.processBufferedData(data, msgs);
				for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
				{
					// Retrieve a packet
					XsDataPacket packet;
					if ((*it).getMessageId() == XMID_MtData) {
						LegacyDataPacket lpacket(1, false);
						lpacket.setMessage((*it));
						lpacket.setXbusSystem(false);
						lpacket.setDeviceId(mtPort.deviceId(), 0);
						lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion,0);	//lint !e534
						XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
					}
					else if ((*it).getMessageId() == XMID_MtData2) {
						packet.setMessage((*it));
						packet.setDeviceId(mtPort.deviceId());
					}
					
					if(packet.containsCalibratedGyroscopeData()&&packet.containsCalibratedAcceleration())
					{
						if(packet.containsPositionLLA())
						{
						  publishOdom(odom_pub,packet);
						}
						else
						{
						  publishImu(imu_pub,packet);
						}					  
					}
					else
					{
						throw std::runtime_error("gyro or accelarator data not found");
					}
				}
				msgs.clear();
				XsTime::msleep(0);
			}
		}
		catch (std::runtime_error const & error)
		{
			sprintf(logger_msg_buffer, "XSENS: %s", error.what());
			ROS_ERROR_STREAM(logger_msg_buffer);
		}
		catch (...)
		{
			ROS_ERROR_STREAM("XSENS: An unknown fatal error has occured. Aborting.");
		}

		// Close port
		std::cout << "Closing port..." << std::endl;
		device.close();
	}
	catch (std::runtime_error const & error)
	{
		sprintf(logger_msg_buffer, "XSENS: %s", error.what());
		ROS_ERROR_STREAM(logger_msg_buffer);
	}
	catch (...)
	{
		ROS_ERROR_STREAM("XSENS: An unknown fatal error has occured. Aborting.");
	}

	std::cout << "EXIT" << std::endl;

	return 0;
}



void publishImu(ros::Publisher &publisher,XsDataPacket &packet)
{
	auto gyro=packet.calibratedGyroscopeData();
	auto acce=packet.calibratedAcceleration();
	
	sensor_msgs::Imu imu;
	imu.header.stamp = ros::Time::now();
	imu.header.frame_id = "imu_link";
	
	//set the angular_velocity				
	imu.angular_velocity.x=gyro[0];
	imu.angular_velocity.y=gyro[1];
	imu.angular_velocity.z=gyro[2];
	
	//set the  linear_acceleration
	imu.linear_acceleration.x=acce[0];
	imu.linear_acceleration.y=acce[1];
	imu.linear_acceleration.z=acce[2];

	//publish the message
	publisher.publish(imu);
}

void BLH2XYZ(XsReal *BLH, double *XYZ)
{
	double B=BLH[0]*PI/180;
	double L=BLH[1]*PI/180;
	double H=BLH[2];
	double a=6378137;
	double e2=0.00669437999013;
	
	double N=a/sqrt(1-e2*sin(B)*sin(B));
	XYZ[0]=(N+H)*cos(B)*cos(L);
	XYZ[1]=(N+H)*cos(B)*sin(L);
	XYZ[2]=(N*(1-e2)+H)*sin(B); 
}

void publishOdom(ros::Publisher &publisher, XsDataPacket &packet)
{
	if(!packet.containsPositionLLA())
		return;
	
	auto quat=packet.orientationQuaternion();
	auto pLLA=packet.positionLLA();
	auto gyro=packet.calibratedGyroscopeData();
	auto acce=packet.calibratedAcceleration();
	
	nav_msgs::Odometry odom;
	
	odom.child_frame_id="base_link";
	odom.header.stamp=ros::Time::now();
	odom.header.frame_id="odom";
	
	odom.pose.pose.orientation.x=quat.x();
	odom.pose.pose.orientation.y=quat.y();
	odom.pose.pose.orientation.z=quat.z();
	odom.pose.pose.orientation.w=quat.w();
	
	double XYZ[3];
	BLH2XYZ(pLLA.toVector().data(),XYZ);
	odom.pose.pose.position.x=XYZ[0];
	odom.pose.pose.position.y=XYZ[1];
	odom.pose.pose.position.z=XYZ[2];

	//set the angular_velocity				
	odom.twist.twist.angular.x=gyro[0];
	odom.twist.twist.angular.y=gyro[1];
	odom.twist.twist.angular.z=gyro[2];

	//set the  linear_acceleration
	odom.twist.twist.linear.x=acce[0];
	odom.twist.twist.linear.y=acce[1];
	odom.twist.twist.linear.z=acce[2];

	publisher.publish(odom);
}
