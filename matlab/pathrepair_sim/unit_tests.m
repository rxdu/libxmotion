% clear workspace
close all;
clear;
clc;

%% create 2d/3d space
sdim = [10,10,5];
s = create_3d_space(sdim);
so = add_obstacle_to_space(s, 0.1);
map = create_2d_map(so);

[x,y,z] = get_index_from_id(s, 1);

% misc space structure helper functions
idx = get_voxel_from_pos(s, [1.1,0.2,1.2]);
idx = get_voxel_from_pos(s, [12.1194,9.4134,4.5]);

%% neighbour finding
nbs = get_2d_neighbours(map, [1,1]);
nbs3 = get_3d_neighbours(s, [2,2,2]);
nbs3s = size(nbs3);

%% 3d sensor model
close all;
ssensor = create_3d_space([10 10 10]);
ssensoro = add_obstacle_to_space(ssensor, 0.1);
display_3d_space(ssensoro);

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