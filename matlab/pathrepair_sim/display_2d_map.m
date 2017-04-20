function display_2d_map(space, path)
    figure
    axis([0 space.x_size 0 space.y_size],'square')    
    hold on
    
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
    
    spath = size(path,2)
    if size(path,2) > 1
%        for i = 1:size(path)-1
%            %wpx = 
%            %wp = space.voxels{}.center_pos;
%            plot(0.5,0.5,'r')
%        end
        for i = 1:size(path,2)-1
            [x1,y1,z1] = get_index_from_id(space, path(i));
            [x2,y2,z2] = get_index_from_id(space, path(i+1));            
            pos1 = space.voxels{x1,y1,z1}.center_pos(1:2);
            pos2 = space.voxels{x2,y2,z2}.center_pos(1:2);
            plot([pos1(1) pos2(1)], [pos1(2) pos2(2)], ...
                '-rs', ...
                'LineWidth',2,...
                'MarkerSize',5,...
                'MarkerEdgeColor','b',...
                'MarkerFaceColor',[0.5,0.5,0.5])
        end
    end
    hold off
end