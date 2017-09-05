function map = create_2d_map(space)
    map = create_3d_space([space.x_size, space.y_size, 1]);
    
    for x = 1:1:space.x_size
        for y = 1:1:space.y_size      
                map.voxels{x,y,1} = space.voxels{x,y,1};  
        end
    end
end