function voxel = create_voxel(x, y, z)
    voxel.id = 0;
    voxel.index = [x, y, z];
    voxel.center_pos = [x-0.5, y-0.5, z-0.5];
    voxel.bound = [x-1:x, y-1:y, z-1:z];
    voxel.occupied = false;
end