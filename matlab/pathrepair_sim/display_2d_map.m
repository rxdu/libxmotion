function display_2d_map(space)
    figure
    axis([0 space.x_size+1 0 space.y_size+1],'square')
    
    
    
    for x = 1:1:space.x_size
        for y = 1:1:space.y_size
                if space.voxels{x,y,1}.occupied == true
                    voxel = space.voxels{x,y,1};
                    
                    x0 = voxel.bound(1);
                    x1 = voxel.bound(2);
                    y0 = voxel.bound(3);
                    y1 = voxel.bound(4);

                    v.Vertices = [x0 y0; ... 
                                  x1 y0; ...
                                  x0 y1; ...
                                  x1 y1];
                    v.Faces = [1 2 4 3];
                    v.EdgeColor = 'k';
                    v.FaceColor = 'g';
                    v.LineWidth = 1;

                    h = patch(v);
                end
        end
    end
end