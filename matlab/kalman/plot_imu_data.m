function plot_imu_data(utime, raw_accel, raw_gyro, raw_magn)
    plot_3axis_sensor(utime, raw_accel, 'accel')
    plot_3axis_sensor(utime, raw_gyro, 'gyro')
    plot_3axis_sensor(utime, raw_magn, 'magn')
end

