function space = create_3d_space(dim)
    space = {};
    
    space.x_size = dim(1);
    space.y_size = dim(2);
    space.z_size = dim(3);
    
    for x = 1:1:space.x_size
        for y = 1:1:space.y_size
            for z = 1:1:space.z_size          
                id = (z - 1) * (space.x_size*space.y_size) + (y - 1) * space.y_size + x - 1;

                voxel = create_voxel(x,y,z);  
                voxel.id = id;
                space.voxels{x,y,z} = voxel;  
            end
        end
    end
end