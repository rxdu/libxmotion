function [path,dist] = search_2d_path(graph2d,s_id,g_id)
    sg = sparse(graph2d);    
    [dist,path,pred] = graphshortestpath(sg,s_id,g_id); 
end