/*
 * test_octomap.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: rdu
 */

#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace octomap;

void print_query_info(point3d query, OcTreeNode* node) {
	if (node != NULL) {
		std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
	}
	else
		std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}

int main(int argc, char* argv[])
{
	OcTree tree (0.1);

	//	std::string tree_path_ = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/octomap/local_3dmap.bt";
	std::string tree_path_ = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/build/bin/test_tree.bt";
	tree.readBinary(tree_path_);

	point3d query (0., 0., 0.);
	OcTreeNode* result = tree.search (query);
	print_query_info(query, result);

	query = point3d(0.,0.,1.);
	result = tree.search (query);
	print_query_info(query, result);

	query = point3d(1.,1.,1.);
	result = tree.search (query);
	print_query_info(query, result);

	std::cout << "\n---------------------------------------------------------\n" << std::endl;
	std::cout << "num of leaf nodes: " << tree.getNumLeafNodes() << std::endl;
	std::cout << "tree depth: " <<  tree.getTreeDepth() << std::endl;

	double mmin[3],mmax[3];

	tree.getMetricMin(mmin[0],mmin[1],mmin[2]);
	tree.getMetricMax(mmax[0],mmax[1],mmax[2]);

	std::cout << "tree bound - min: \n" << mmin[0] << " , " << mmin[1] << " , " << mmin[2] << std::endl;
	std::cout << "tree bound - max: \n" << mmax[0] << " , " << mmax[1] << " , " << mmax[2] << std::endl;

	uint32_t idx = 0;
	for(auto it = tree.begin_leafs(tree.getTreeDepth()); it != tree.end_leafs(); it++)
	{
		//std::cout << (*it).getOccupancy() << std::endl;
		idx++;
	}
	std::cout << "leaf checked: " << idx << std::endl;
}


