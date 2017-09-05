function obs_space = add_obstacle_to_space(space,obs_percentage)
    obs_space = space;
    
    btn_layer_num = space.x_size * space.y_size;
    obs_num = int64(btn_layer_num * obs_percentage);
    
    pos_rn = rand(obs_num,1);    
    hei_num_rn = rand(obs_num,1);    
    btn_idx = int64(pos_rn * btn_layer_num);
        
    for i = 1:size(btn_idx,1)
        idx = btn_idx(i);
        z = 1;
        y = int64(idx/space.y_size) + 1;
        x = int64(mod(idx,space.y_size)) + 1;        
        
        % make sure voxel 1 and space.x_size*space.y_size are not occupied
        if ((x == 1) & (y == 1)) | (x == space.x_size) & (y == space.y_size) 
            continue;
        end
        
        obs_space.voxels{x,y,z}.occupied = true;
        
        % randomly set obstacles along z axis
        hei_rn = rand(int64(space.z_size*hei_num_rn(i)),1);
        hei = int64(hei_rn*space.z_size);
        for hidx = 1:size(hei,1)
            h = hei(hidx);
            if h <= 0
                h = 1;
            end
            if h > space.z_size
                h = space.z_size;
            end
            %if h <= space.z_size*2/3
                obs_space.voxels{x,y,h}.occupied = true;
            %end
        end
    end
    
    
    obs_space.voxels{1,1,1}.occupied = false;
    obs_space.voxels{space.x_size,space.y_size,1}.occupied = false;
end