function var = calc_allan_variance(gyro_data)
    var = zeros(size(gyro_data,1),3);
    
    K = size(gyro_data,1);
    sum = zeros(1,3);
    for i = 2:K 
        sum = sum + (gyro_data(i,2:4) - gyro_data(i-1,2:4)).^2; 
        var(i,:) = sum./(2*i);
    end
end