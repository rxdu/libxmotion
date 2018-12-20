function [utime,raw_accel,raw_gyro,raw_magn] = read_imu_log(log_file)
    imu_data = csvread(log_file);

    % structure of data frame
    % 1 utime, 
    % 2-4 mpu9250_data_.accel[0], mpu9250_data_.accel[1], mpu9250_data_.accel[2],
    % 5-7 mpu9250_data_.gyro[0] * DEG_TO_RAD, mpu9250_data_.gyro[1] * DEG_TO_RAD, mpu9250_data_.gyro[2] * DEG_TO_RAD,
    % 8-10 mpu9250_data_.mag[0], mpu9250_data_.mag[1], mpu9250_data_.mag[2]
    utime = imu_data(:,1);
    raw_accel = imu_data(:,2:4);
    raw_gyro = imu_data(:,5:7);
    raw_magn = imu_data(:,8:10);
end

