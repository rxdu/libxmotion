function neighbours = get_3d_neighbours(space, voxel_idx)
    neighbours = [];

    x = voxel_idx(1);
    y = voxel_idx(2);
    z = voxel_idx(3);

    nidx = [];
    for xi = x-1:1:x+1
        for yi = y-1:1:y+1
            for zi = z-1:1:z+1
                if not (xi == x && yi == y && zi == z) && ...
                    xi <= space.x_size && xi > 0 && ...
                        yi <= space.y_size && yi > 0 && ...
                           zi <= space.z_size && zi > 0
                    err = space.voxels{xi,yi,zi}.center_pos - ...
                        space.voxels{x,y,z}.center_pos;
                    cost = norm(err);
                    nidx = [nidx; xi, yi, zi, cost];
                end
            end
        end
    end

    for i = 1:size(nidx,1)
        if space.voxels{nidx(i,1), nidx(i,2)}.occupied == false
            nidx(i,:)
            neighbours = [neighbours; nidx(i,:)];
        end
    end
end