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
	//std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(0.1);

//	point3d endpoint ((float) 0.05f, (float) 0.05f, (float) 0.05f);
//	tree.updateNode(endpoint, true); // integrate 'occupied' measurement
//
//	point3d endpoint2 (0.05f, 0.05f, 0.15f);
//	tree.updateNode(endpoint2, false);  // integrate 'free' measurement

	//	std::string tree_path_ = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/octomap/local_3dmap.bt";
	//std::string tree_path_ = "/home/rdu/Workspace/srcl_rtk/librav/build/bin/test_tree.bt";
	std::string tree_path_ = "/home/rdu/Workspace/srcl_rtk/librav/build/bin/simple_tree.bt";
	tree.readBinary(tree_path_);

	//tree.writeBinary("test_octomap_tree.bt");

//	point3d query (0., 0., 0.);
//	OcTreeNode* result = tree.search (query);
//	print_query_info(query, result);
//
//	query = point3d(0.,0.,1.);
//	result = tree.search (query);
//	print_query_info(query, result);
//
//	query = point3d(1.,1.,1.);
//	result = tree.search (query);
//	print_query_info(query, result);

	std::cout << "\n*********************************************************\n" << std::endl;
	std::cout << "num of leaf nodes: " << tree.getNumLeafNodes() << std::endl;
	std::cout << "tree depth: " <<  tree.getTreeDepth() << std::endl;

	double mmin[3],mmax[3];

	tree.getMetricMin(mmin[0],mmin[1],mmin[2]);
	tree.getMetricMax(mmax[0],mmax[1],mmax[2]);

	std::cout << "tree bound - min: \n" << mmin[0] << " , " << mmin[1] << " , " << mmin[2] << std::endl;
	std::cout << "tree bound - max: \n" << mmax[0] << " , " << mmax[1] << " , " << mmax[2] << std::endl;

	std::cout << "\n---------------------------------------------------------\n" << std::endl;
	uint32_t idx = 0;
	for(auto it = tree.begin_leafs(tree.getTreeDepth()); it != tree.end_leafs(); it++)
	{
//		std::cout << (*it).getOccupancy() << std::endl;
		//		if((*it).getOccupancy() < 0.95) {
//		std::cout << "size: " << it.getSize() << std::endl;
//		std::cout << "coordinate: " << it.getCoordinate() << std::endl;
		idx++;
		//		}

		//		if(it.getSize() == 0.2)
		//			idx++;
//		std::cout << std::endl;
	}
	std::cout << "leaf checked: " << idx << std::endl;
}


