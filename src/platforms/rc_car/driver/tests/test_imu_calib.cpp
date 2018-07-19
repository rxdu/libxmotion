#include <iostream>
#include <cstdint>
#include <vector>

#include "common/librav_types.hpp"
#include "utility/librav_utility.h"
#include "sensor/imu_filter.h"

using namespace librav;

int main()
{
    int64_t time_stamp;
    double gyro[3];
    double accel[3];
    io::CSVReader<7> in("/home/rdu/Workspace/imu_calib/data/raw_imu.20171111235018.data");

    std::vector<AccGyroData> raw_data;
    while (in.read_row(time_stamp, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]))
    {
        // std::cout << "data: " << time_stamp << " , " << gyro[0] << " , " << gyro[1] << " , " << gyro[2] << " , "
        //         << accel[0] << " , " << accel[1] << " , " << accel[2] << std::endl;
        raw_data.push_back(AccGyroData(time_stamp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]));
    }

    IMUFilter filter;
    std::shared_ptr<CsvLogger> calib_logger_;
    calib_logger_ = std::make_shared<CsvLogger>("calib_imu", "/home/rdu/CarLog");
    std::vector<AccGyroData> cor_data;
    for(auto& dt : raw_data)
    {
        auto cor_dt = 
            filter.CorrectIMURawData(AccGyroData(dt.mtime, dt.accel.x, dt.accel.y, dt.accel.z, 
                                dt.gyro.x, dt.gyro.y, dt.gyro.z));
        calib_logger_->LogData(cor_dt.mtime, cor_dt.accel.x, cor_dt.accel.y, cor_dt.accel.z, 
                                cor_dt.gyro.x, cor_dt.gyro.y, cor_dt.gyro.z);
    }
}