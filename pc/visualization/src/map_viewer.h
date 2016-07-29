/*
 * map_viewer.h
 *
 *  Created on: Jul 26, 2016
 *      Author: rdu
 */

#ifndef VISUALIZATION_SRC_MAP_VIEWER_H_
#define VISUALIZATION_SRC_MAP_VIEWER_H_

// C++ standard headers
#include <vector>
#include <memory>

// OpenCV headers
#include "opencv2/opencv.hpp"

// User headers
#include "graph/graph.h"
#include "graph_vis/graph_vis.h"
#include "square_grid/square_grid.h"
#include "quadtree/quad_tree.h"
#include "map/map_info.h"
#include "map/map_type.h"

namespace srcl_ctrl {

enum class CellDecompMethod {
    SQUARE_GRID,
    QUAD_TREE,
	NOT_SPECIFIED
};

typedef struct {
	CellDecompMethod method;
	uint8_t qtree_depth;
	uint64_t square_cell_size;
} DecomposeConfig;

class MapViewer {
public:
	MapViewer();
	~MapViewer();

private:
	// images
	cv::Mat raw_image_;
	cv::Mat displayed_image_;

	Map_t<SquareGrid> sg_map_;
	Map_t<QuadTree> qt_map_;

	std::shared_ptr<Graph_t<SquareCell*>> sgrid_graph_;
	std::shared_ptr<Graph_t<QuadTreeNode*>> qtree_graph_;

	GraphVis graph_vis_;

	// workspace decomposition
	bool image_updated_;
	uint64_t squarecell_size_;
	uint8_t qtree_depth_;
	CellDecompMethod active_decompose_;

public:
	//cv::Mat GetLoadedMap() const {return raw_image_;}
	bool HasMapLoaded() const {
		if(raw_image_.data)
			return true;
		else
			return false;
	}

public:
	bool ReadMapFromFile(std::string file_name);
	void SaveResultToFile(std::string file_name);
	cv::Mat DecomposeWorkspace(DecomposeConfig config, MapInfo &info);
	cv::Mat HighlightSelectedNode(uint32_t x, uint32_t y);
};

}



#endif /* VISUALIZATION_SRC_MAP_VIEWER_H_ */
