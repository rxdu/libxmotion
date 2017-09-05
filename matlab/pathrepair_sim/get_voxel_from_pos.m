function idx = get_voxel_from_pos(space, pos)
    idx = floor(pos) + 1;
    
    if idx(1) < 1
        idx(1) = 1;
    end
    if idx(1) > space.x_size
        idx(1) = space.x_size;
    end
    
    if idx(2) < 1
        idx(2) = 1;
    end
    if idx(2) > space.y_size
        idx(2) = space.y_size;
    end
    
    if idx(3) < 1
        idx(3) = 1;
    end
    if idx(3) > space.z_size
        idx(3) = space.z_size;
    end
end