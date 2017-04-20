% clear workspace
close all;
clear;
clc;

%% create 2d/3d space
sdim = [50,50,5];
s = create_3d_space(sdim);
so = add_obstacle_to_space(s, 0.1);
map = create_2d_map(so);

[x,y,z] = get_index_from_id(s, 1);

%% neighbour finding
nbs = get_2d_neighbours(map, [1,1]);

%% graph construction
ss = create_3d_space([50, 50, 5]);
sso = add_obstacle_to_space(ss, 0.2);
smap = create_2d_map(sso);
sgraph = create_graph(smap);

%% graph search
[path,dist] = search_2d_path(sgraph,1,smap.x_size*smap.y_size);

% h = view(biograph(sgraph,[],'ShowWeights','on'))
% set(h.Nodes(path),'Color',[1 0.4 0.4])
% edges = getedgesbynodeid(h,get(h.Nodes(path),'ID'));
% set(edges,'LineColor',[1 0 0])
% set(edges,'LineWidth',1.5)
path;

%% display
display_2d_map(smap,path)
%display_3d_space(so)