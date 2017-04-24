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
        sensor_cor = sensor_data.sensor_model.sensor_info.sensor_pos;
        ll_pos = sensor_data.sensor_model.projection_plane_points(1,:);
        rl_pos = sensor_data.sensor_model.projection_plane_points(2,:);
        llu_pos = sensor_data.sensor_model.projection_plane_points(3,:);
        rlu_pos = sensor_data.sensor_model.projection_plane_points(4,:);
        lll_pos = sensor_data.sensor_model.projection_plane_points(5,:);
        rll_pos = sensor_data.sensor_model.projection_plane_points(6,:);
        
        % projection plane
        plot3([ll_pos(1) rl_pos(1)],[ll_pos(2) rl_pos(2)],[ll_pos(3) rl_pos(3)],'b')
        plot3([lll_pos(1) llu_pos(1) rlu_pos(1) rll_pos(1) lll_pos(1)],...
              [lll_pos(2) llu_pos(2) rlu_pos(2) rll_pos(2) lll_pos(2)],...
              [lll_pos(3) llu_pos(3) rlu_pos(3) rll_pos(3) lll_pos(3)],'b')

        % boundary points
        plot3(ll_pos(1),ll_pos(2),ll_pos(3),'b*')
        plot3(rl_pos(1),rl_pos(2),rl_pos(3),'b*')

        plot3(llu_pos(1),llu_pos(2),llu_pos(3),'b*')
        plot3(rlu_pos(1),rlu_pos(2),rlu_pos(3),'b*')

        plot3(lll_pos(1),lll_pos(2),lll_pos(3),'b*')
        plot3(rll_pos(1),rll_pos(2),rll_pos(3),'b*')

        % sensor to boundaries
        plot3([ll_pos(1) sensor_cor(1)],[ll_pos(2) sensor_cor(2)],[ll_pos(3) sensor_cor(3)],'b')
        plot3([rl_pos(1) sensor_cor(1)],[rl_pos(2) sensor_cor(2)],[rl_pos(3) sensor_cor(3)],'b')

        plot3([llu_pos(1) sensor_cor(1)],[llu_pos(2) sensor_cor(2)],[llu_pos(3) sensor_cor(3)],'b')
        plot3([rlu_pos(1) sensor_cor(1)],[rlu_pos(2) sensor_cor(2)],[rlu_pos(3) sensor_cor(3)],'b')

        plot3([lll_pos(1) sensor_cor(1)],[lll_pos(2) sensor_cor(2)],[lll_pos(3) sensor_cor(3)],'b')
        plot3([rll_pos(1) sensor_cor(1)],[rll_pos(2) sensor_cor(2)],[rll_pos(3) sensor_cor(3)],'b')
    end
    hold off
end