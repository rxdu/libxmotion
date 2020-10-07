/* 
 * sensor_types.hpp
 * 
 * Created on: Oct 10, 2018 11:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SENSOR_TYPES_HPP
#define SENSOR_TYPES_HPP

#include <cstdint>
#include <vector>
#include <iostream>

#include "eigen3/Eigen/Core"

#include "adtypes/types/base_types.hpp"

namespace autodrive
{

/****************** Types for Sensors ******************/
struct IMU9DOFData
{
	IMU9DOFData() : mtime(0),
					accel(Point3d(0, 0, 0)),
					gyro(Point3d(0, 0, 0)),
					magn(Point3d(0, 0, 0)){};

	IMU9DOFData(int64_t time, double accel_x, double accel_y, double accel_z,
				double gyro_x, double gyro_y, double gyro_z,
				double magn_x, double magn_y, double magn_z) : mtime(time),
															   accel(Point3d(accel_x, accel_y, accel_z)),
															   gyro(Point3d(gyro_x, gyro_y, gyro_z)),
															   magn(Point3d(magn_x, magn_y, magn_z)){};

	TimeStamp mtime;

	Point3d gyro;
	Point3d accel;
	Point3d magn;

	friend std::ostream &operator<<(std::ostream &os, const IMU9DOFData &data)
	{
		os << "time_stamp: " << data.mtime
		   << " ; accel(x,y,z): " << data.accel.x << " , " << data.accel.y << " , " << data.accel.z
		   << " ; gyro(x,y,z): " << data.gyro.x << " , " << data.gyro.y << " , " << data.gyro.z
		   << " ; magn(x,y,z): " << data.magn.x << " , " << data.magn.y << " , " << data.magn.z << std::endl;
		return os;
	}
};

struct IMU6DOFData
{
	IMU6DOFData() : mtime(0),
					accel(Point3f(0, 0, 0)),
					gyro(Point3f(0, 0, 0)){};

	IMU6DOFData(int64_t time, double accel_x, double accel_y, double accel_z,
				double gyro_x, double gyro_y, double gyro_z) : mtime(time),
															   accel(Point3f(accel_x, accel_y, accel_z)),
															   gyro(Point3f(gyro_x, gyro_y, gyro_z)){};

	TimeStamp mtime;
	Point3f accel;
	Point3f gyro;

	friend std::ostream &operator<<(std::ostream &os, const IMU6DOFData &data)
	{
		os << "time_stamp: " << data.mtime
		   << " ; accel(x,y,z): " << data.accel.x << " , " << data.accel.y << " , " << data.accel.z
		   << " ; gyro(x,y,z): " << data.gyro.x << " , " << data.gyro.y << " , " << data.gyro.z << std::endl;
		return os;
	}
};

struct IMUCalibParams
{
	Eigen::Matrix<double, 3, 3> misalignment_matrix;
	Eigen::Matrix<double, 3, 3> scale_matrix;
	Eigen::Matrix<double, 3, 1> bias_vector;
};
} // namespace autodrive

#endif /* SENSOR_TYPES_HPP */
