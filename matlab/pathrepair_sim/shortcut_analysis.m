function ays_res = shortcut_analysis(map, graph, rng, goal)
    ays_res.map = map;
    
    sg = sparse(graph);    
    %[dist,path,pred] = graphshortestpath(sg,s_id,g_id); 
    %usg = tril(sg + sg');

    asp = graphallshortestpaths(sg,'directed',false);
   
    dist_to_goal = asp(end,:);
    idx = 1;
    for z = 1:1:ays_res.map.z_size
        for y = 1:1:ays_res.map.y_size
            for x = 1:1:ays_res.map.x_size
                ays_res.map.voxels{x,y,z}.cost_to_goal = dist_to_goal(idx);
                idx = idx + 1;
            end
        end
    end
    
    %calc_direct_dist([1 1], [2 3])
    %evaluate_cell(ays_res.map, [1 1 1], 3)
    rewards_max = [];
    for z = 1:1:ays_res.map.z_size
        for y = 1:1:ays_res.map.y_size
            for x = 1:1:ays_res.map.x_size
                if ays_res.map.voxels{x,y,z}.occupied == true
                    continue
                end
                
                [r, a] = evaluate_cell(ays_res.map, [x y z], rng);
                ays_res.map.voxels{x,y,z}.shortcut_rewards = r;
                ays_res.map.voxels{x,y,z}.shortcut_heading = a;
                rewards_max = [rewards_max r];
            end
        end
    end
    
    ays_res.max_reward = max(rewards_max(:));
    
    %% helper functions
    function dist = calc_direct_dist(start, goal)
        if start(1) > goal(1)
            x_err = start(1) - goal(1);
        else
            x_err = goal(1) -  start(1);
        end
        
        if start(2) > goal(2)
            y_err = start(2) - goal(2);
        else
            y_err = goal(2) -  start(2);
        end
        
        diag_steps = 0;
        straight_steps = 0;
        
        if(x_err > y_err)
            diag_steps = y_err;
            straight_steps = x_err - y_err;            
        else
            diag_steps = x_err;
            straight_steps = y_err - x_err;
        end
        
        dist = diag_steps * sqrt(2) + straight_steps;
    end

    function nbs = get_neibours_within_range(map, cell, range)
        nbs = [];
        xi = cell(1);
        yi = cell(2);
        
        if xi <= range
            xmin = 1;
        else
            xmin = xi - range;
        end
        if xi + range >= map.x_size;
            xmax = map.x_size;
        else
            xmax = xi + range;
        end
        
        if yi <= range
            ymin = 1;
        else
            ymin = yi - range;
        end
        if yi + range >= map.y_size;
            ymax = map.y_size;
        else
            ymax = yi + range;
        end
        
        for i = xmin:1:xmax
            for j = ymin:1:ymax
                if i == xi && j == yi
                    continue
                end
                nbs = [nbs; i j];
            end
        end
    end

    function [sc_reward, sc_angle] = evaluate_cell(map, cell, range)
        nbs = get_neibours_within_range(map, cell, range);
        
        % calculate max rewards
        rws = [];
        for i = 1:size(nbs,1)
            if map.voxels{nbs(i,1), nbs(i,2), 1}.occupied == true
                continue
            end
            
            rwd = map.voxels{cell(1), cell(2), cell(3)}.cost_to_goal - ...
                map.voxels{nbs(i,1), nbs(i,2), 1}.cost_to_goal - ...
                calc_direct_dist([cell(1), cell(2), cell(3)], [nbs(i,1), nbs(i,2), 1]);
            rws = [rws; rwd i];
        end
        
        [m, mi] = max(rws(:,1), [], 1);
        
        % calculate desired heading
        %nbs(mi, :)
        max_rwd_cell = map.voxels{nbs(mi, 1),nbs(mi, 2), 1};
        max_rwd_vec = max_rwd_cell.center_pos;
        
        pos_vec = map.voxels{cell(1), cell(2), cell(3)}.center_pos;
        dir_vec = max_rwd_vec - pos_vec;
        
        x_vec = [0 -1 0];
        y_vec = [-1 0 0];
        
        x_dir_vec = dot(dir_vec, x_vec);
        y_dir_vec = dot(dir_vec, y_vec);
        
        if y_dir_vec > 0
            angle = acos(dot(dir_vec/norm(dir_vec), x_vec));
        elseif y_dir_vec < 0
            angle = -acos(dot(dir_vec/norm(dir_vec), x_vec));   
        else
            if x_dir_vec > 0
                angle = 0;
            else
                angle = pi;
            end
        end
        
        sc_reward = m;
        sc_angle = angle/pi*180;
    end
end