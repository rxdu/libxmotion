function sspace = shortcut_analysis(space, graph, rng, goal)
    sspace = space;
    
    sg = sparse(graph);    
    %[dist,path,pred] = graphshortestpath(sg,s_id,g_id); 
    usg = tril(sg + sg');

    asp = graphallshortestpaths(usg,'directed',false);
    asps = size(asp(end,:))
    asp(end,:)
end