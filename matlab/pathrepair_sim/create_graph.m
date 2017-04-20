function graph = create_graph(space)    
    xs = space.x_size;
    ys = space.y_size;
    zs = space.z_size;
    
    graph = zeros(xs*ys*zs, xs*ys*zs);
    
    for x = 1:1:xs
        for y = 1:1:ys
            for z = 1:1:zs                 
                neighbours = [];
                if zs == 1
                    neighbours = get_2d_neighbours(space, [x,y]);
                else
                    neighbours = get_3d_neighbours(space, [x,y,z]);
                end
                
                for i = 1:size(neighbours,1)
                    n = neighbours(i,:);
                    nx = int64(n(1));
                    ny = int64(n(2));
                    nz = int64(n(3));
                    graph(space.voxels{x,y,z}.id, space.voxels{nx,ny,nz}.id) = n(4);
                end
            end
        end
    end
end