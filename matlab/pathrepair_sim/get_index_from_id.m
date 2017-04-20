function [x,y,z] = get_index_from_id(space, id)
    xs = space.x_size;
    ys = space.y_size;
    zs = space.z_size;
    
    z = floor((id-1) / (xs*ys)) + 1;
    remz = id - (z - 1)*zs;
    y = floor((remz-1) / xs) + 1;
    x = id - (z - 1)*zs - (y-1)*xs;
end