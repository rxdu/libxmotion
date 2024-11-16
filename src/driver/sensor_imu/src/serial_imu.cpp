//serial_imu.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <serial_imu/Imu_0x91_msg.h>
#include <serial_imu/Imu_0x62_msg.h>
#include <serial_imu/Imu_data_package.h>
#include <signal.h>

#ifdef __cplusplus 
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "ch_serial.h"

#define IMU_SERIAL   "/dev/ttyUSB0"
#define BAUD         (115200)
#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     1024

void publish_0x91_data(raw_t *data, serial_imu::Imu_0x91_msg *data_imu);
void publish_imu_data(raw_t *data, sensor_msgs::Imu *imu_data);
void publish_0x62_data(raw_t *data, serial_imu::Imu_0x62_msg *data_imu);

#ifdef __cplusplus
}
#endif

static raw_t raw;

static int frame_rate;
static int frame_count;

static uint8_t buf[2048];

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}

int main(int argc, char** argv)
{
	int rev = 0;
	ros::init(argc, argv, "serial_imu");
	ros::NodeHandle n;

	ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("/IMU_data", 20);
	ros::Publisher Imu_0x91_pub = n.advertise<serial_imu::Imu_0x91_msg>("/imu_0x91_package", 10);
	ros::Publisher Imu_0x62_pub = n.advertise<serial_imu::Imu_0x62_msg>("/imu_0x62_package", 10);

	serial::Serial sp;

	serial::Timeout to = serial::Timeout::simpleTimeout(100);

	sp.setPort(IMU_SERIAL);

	sp.setBaudrate(BAUD);

	sp.setTimeout(to);
	
	signal(SIGALRM,timer);

	try
	{
		sp.open();
	}
	catch(serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}
    
	if(sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
	}
	else
	{
		return -1;
	}
	
	alarm(1);
	
	ros::Rate loop_rate(500);
	sensor_msgs::Imu imu_data;
	serial_imu::Imu_0x91_msg imu_0x91_msg;
	serial_imu::Imu_0x62_msg imu_0x62_msg;

	while(ros::ok())
	{
		size_t num = sp.available();
		if(num!=0)
		{
			uint8_t buffer[BUF_SIZE]; 
	
			if(num > BUF_SIZE)
				num = BUF_SIZE;
			
			num = sp.read(buffer, num);
			if(num > 0)
			{
				imu_data.header.stamp = ros::Time::now();
				imu_data.header.frame_id = "base_link";

				imu_0x91_msg.header.stamp = ros::Time::now(); 
				imu_0x91_msg.header.frame_id = "base_0x91_link";

				imu_0x62_msg.header.stamp = ros::Time::now();
				imu_0x62_msg.header.frame_id = "base_0x62_link";

				for (int i = 0; i < num; i++)
				{
					rev = ch_serial_input(&raw, buffer[i]);
				
					if(raw.item_code[raw.nitem_code - 1] != KItemGWSOL)
					{
						if(rev)
						{
							frame_count++;

							publish_0x91_data(&raw, &imu_0x91_msg);
							Imu_0x91_pub.publish(imu_0x91_msg);

							publish_imu_data(&raw, &imu_data);
							IMU_pub.publish(imu_data);

						}
					}
					else
					{
						if(rev)
						{
							frame_count++;

							Imu_0x62_pub.publish(imu_0x62_msg);
							publish_0x62_data(&raw, &imu_0x62_msg);
						}
					}
				}
			}
		}
		loop_rate.sleep();
	}
    
	sp.close();
 
	return 0;
}

void imu_data_package(raw_t *data, serial_imu::Imu_data_package *data_imu, int num)
{
	data_imu->tag = data->item_code[num];

	data_imu->frame_rate = frame_rate;

	data_imu->id = data->imu[num].id;

	data_imu->time = data->imu[num].timestamp;

	data_imu->prs = data->imu[num].pressure;

	data_imu->acc_x = data->imu[num].acc[0];
	data_imu->acc_y = data->imu[num].acc[1];
	data_imu->acc_z = data->imu[num].acc[2];

	data_imu->gyr_x = data->imu[num].gyr[0];
	data_imu->gyr_y = data->imu[num].gyr[1];
	data_imu->gyr_z = data->imu[num].gyr[2];

	data_imu->mag_x = data->imu[num].mag[0];
	data_imu->mag_y = data->imu[num].mag[1];
	data_imu->mag_z = data->imu[num].mag[2];

	data_imu->eul_r = data->imu[num].eul[0];
	data_imu->eul_p = data->imu[num].eul[1];
	data_imu->eul_y = data->imu[num].eul[2];

	data_imu->quat_w = data->imu[num].quat[0];
	data_imu->quat_x = data->imu[num].quat[1];
	data_imu->quat_y = data->imu[num].quat[2];
	data_imu->quat_z = data->imu[num].quat[3];
}

void publish_0x91_data(raw_t *data, serial_imu::Imu_0x91_msg *data_imu)
{
	imu_data_package(data, &(data_imu->imu_data),data->nimu - 1);
}


void publish_imu_data(raw_t *data, sensor_msgs::Imu *imu_data)
{	
	imu_data->orientation.x = data->imu[data->nimu - 1].quat[1];
	imu_data->orientation.y = data->imu[data->nimu - 1].quat[2];
	imu_data->orientation.z = data->imu[data->nimu - 1].quat[3];
	imu_data->orientation.w = data->imu[data->nimu - 1].quat[0];
	imu_data->angular_velocity.x = data->imu[data->nimu - 1].gyr[0] * DEG_TO_RAD;
	imu_data->angular_velocity.y = data->imu[data->nimu - 1].gyr[1] * DEG_TO_RAD;
	imu_data->angular_velocity.z = data->imu[data->nimu - 1].gyr[2] * DEG_TO_RAD;
	imu_data->linear_acceleration.x = data->imu[data->nimu - 1].acc[0] * GRA_ACC;
	imu_data->linear_acceleration.y = data->imu[data->nimu - 1].acc[1] * GRA_ACC;
	imu_data->linear_acceleration.z = data->imu[data->nimu - 1].acc[2] * GRA_ACC;
}


void publish_0x62_data(raw_t *data, serial_imu::Imu_0x62_msg *data_imu)
{
#if 1
	/*  */
	data_imu->tag = data->item_code[data->nitem_code];
	data_imu->gw_id = data->gwid;
	data_imu->node_num = data->nimu;

	for (int i = 0; i < data_imu->node_num; i++)
		imu_data_package(data, &(data_imu->node_data[i]), i);
#endif		
	
}

