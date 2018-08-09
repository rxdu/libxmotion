close all;
clear;
clc;

%log_file = "/home/rdu/mp.20180809011824.data"
%log_file = "/home/rdu/mp_trans.20180807092633.data"
%draw_primitives_with_path(log_file, 6)

%log_file = "/home/rdu/path.20180808023529.data"
%draw_path(log_file, 12)

%graph_file = "/home/rdu/graph.20180809011250.data"
%draw_lattice_graph(graph_file,6)

search_file = "/home/rdu/search.20180809031924.data"

draw_lattice_graph(search_file, 6)
