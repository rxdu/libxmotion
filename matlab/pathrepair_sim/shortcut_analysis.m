function ays_res = shortcut_analysis(map, graph, rng, goal)
    ays_res.map = map;
    
    sg = sparse(graph);    
    %[dist,path,pred] = graphshortestpath(sg,s_id,g_id); 
    %usg = tril(sg + sg');

    asp = graphallshortestpaths(sg,'directed',false);
   
    dist_to_goal = asp(end,:);
    idx = 1;
    for z = 1:1:map.z_size
        for y = 1:1:map.y_size
            for x = 1:1:map.x_size
                ays_res.map.voxels{x,y,z}.cost_to_goal = dist_to_goal(idx);
                idx = idx + 1;
            end
        end
    end
    
    calc_direct_dist([1 1], [2 3])
    
    % helper functions
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
end