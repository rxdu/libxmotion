function [outputArg1,outputArg2] = build_gaussian_field(positions,velocities)
    %BUILD_GAUSSIAN_FIELD Summary of this function goes here
    %   Detailed explanation goes here
    
    outputArg1 = positions;
    outputArg2 = velocities;
    
    step = 0.1;
    [fx,fy] = meshgrid(-5:step:5);
    fz = zeros(size(fx,1),size(fy,2));

    V = 0.5;
    for i = 1:size(fx,1)
        for j = 1:size(fy,2)
            fz(i,j) = expm(-(fx(i,j)*fx(i,j) + fy(i,j)*fy(i,j))/(2*V))/(2*pi*V);
        end
    end
    
    surf(fx,fy,fz)
    
end

