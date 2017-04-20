function display_voxel(voxel, edge_color, face_color, face_alpha)
    % set default values
    if nargin == 1
        edge_color = 'k';
        show_face = false;
    elseif nargin == 2
        show_face = false;
    elseif nargin == 3
        show_face = true;
        face_alpha = 0.3;
    else
        show_face = true;
    end

    x0 = voxel.bound(1);
    x1 = voxel.bound(2);
    y0 = voxel.bound(3);
    y1 = voxel.bound(4);
    z0 = voxel.bound(5);
    z1 = voxel.bound(6);
    
    v.Vertices = [x0 y0 z0; ... 
                  x0 y0 z1; ...
                  x0 y1 z0; ...
                  x0 y1 z1; ...
                  x1 y0 z0; ...
                  x1 y0 z1; ...
                  x1 y1 z0; ...
                  x1 y1 z1];
    v.Faces = [1 2 4 3; 1 2 6 5; 1 3 7 5; 5 6 8 7; 2 4 8 6; 3 4 8 7];
    v.EdgeColor = edge_color;
    if show_face == true
        v.FaceColor = face_color;
    else
        v.FaceColor = 'none';
    end
    v.LineWidth = 1;
        
    h = patch(v);
    if show_face == true
        set(h,'FaceAlpha',face_alpha);
    end
end