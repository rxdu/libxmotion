function neighbours = get_2d_neighbours(map, cell_idx)
    neighbours = [];
    
    x = cell_idx(1);
    y = cell_idx(2);
    z = 1;
    
    diag_cost = sqrt(2);
    stra_cost = 1;
    
    nidx = [x-1, y-1, z, diag_cost;...
        x-1, y, z, stra_cost;...
        x-1, y+1, z, diag_cost; ...
        x, y-1, z, stra_cost; ...
        x, y+1, z, stra_cost; ...
        x+1, y-1, z, diag_cost; ...
        x+1, y, z, stra_cost; ...
        x+1, y+1, z, diag_cost];
    
    for i = 1:size(nidx,1)
       if nidx(i,1) <= map.x_size && nidx(i,1) > 0 && ... 
           nidx(i,2) <= map.y_size && nidx(i,2) > 0 
            if map.voxels{nidx(i,1), nidx(i,2)}.occupied == false
                neighbours = [neighbours; nidx(i,:)];
            end
       end
    end
end