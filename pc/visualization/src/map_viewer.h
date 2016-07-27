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
#include "square_grid/square_grid.h"
#include "quadtree/quad_tree.h"

namespace srcl_ctrl {

enum class CellDecompMethod {
    SQUARE_GRID,
    QUAD_TREE
};

typedef struct {
	CellDecompMethod method;
	uint8_t qtree_depth;
	uint64_t square_cell_size;
	bool show_padded_area;
} DecomposeConfig;

class MapViewer {
public:
	MapViewer();
	~MapViewer();

private:
	// images
	cv::Mat raw_image_;
	cv::Mat map_image_;

	// workspace decomposition
	bool disp_once;
	CellDecompMethod cell_decomp_method_;
	uint8_t qtree_depth_;

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
	cv::Mat DecomposeWorkspace(DecomposeConfig config);
};

}



#endif /* VISUALIZATION_SRC_MAP_VIEWER_H_ */
