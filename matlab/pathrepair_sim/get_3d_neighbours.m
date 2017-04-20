function neighbours = get_3d_neighbours(map, cell_idx)
    neighbours = [];
    
    x = cell_idx(1);
    y = cell_idx(2);
    
    diag_cost = sqrt(2);
    stra_cost = 1;
    
    nidx = [x-1, y-1, diag_cost;...
        x-1, y, stra_cost;...
        x-1, y+1, diag_cost; ...
        x, y-1, stra_cost; ...
        x, y+1, stra_cost; ...
        x+1, y-1, diag_cost; ...
        x+1, y, stra_cost; ...
        x+1, y+1, diag_cost];
    
    for i = 1:size(nidx,1)
       if nidx(i,1) <= map.x_size && nidx(i,1) > 0 && ... 
           nidx(i,2) <= map.y_size && nidx(i,2) > 0 
            if map.voxels{nidx(i,1), nidx(i,2)}.occupied == false
                nidx(i,:)
                neighbours = [neighbours; nidx(i,:)];
            end
       end
    end
end