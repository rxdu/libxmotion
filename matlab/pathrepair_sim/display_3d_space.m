function display_3d_space(space, sensor_data)
    if nargin == 1
        show_sensor = false;
    elseif nargin == 2
        show_sensor = true;
        sd = sensor_data;
    end
    
    figure
    view(3)
    hold on
    max_lim = max([space.x_size space.y_size space.z_size]);
    axis([0 max_lim 0 max_lim 0 max_lim],'square')
    
    for x = 1:1:space.x_size
        for y = 1:1:space.y_size
            for z = 1:1:space.z_size          
                if space.voxels{x,y,z}.occupied == true
                    display_voxel(space.voxels{x,y,z}, 'k', 'g')
                end
            end
        end
    end
    
    if show_sensor
        sensor_center = space.voxels{sd.sensor_pos(1),sd.sensor_pos(2), sd.sensor_pos(3)};
        plot3(sensor_center.center_pos(1),sensor_center.center_pos(2),sensor_center.center_pos(3),'r*')
    end
    hold off
end