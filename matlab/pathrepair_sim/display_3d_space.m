function display_3d_space(space)
    view(3)
    axis([0 space.x_size+1 0 space.y_size+1 0 space.z_size+1],'square')
    
    for x = 1:1:space.x_size
        for y = 1:1:space.y_size
            for z = 1:1:space.z_size          
                if space.voxels{x,y,z}.occupied == true
                    display_voxel(space.voxels{x,y,z}, 'k', 'g')
                end
            end
        end
    end
end