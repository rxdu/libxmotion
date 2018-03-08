/*
 * graph_vis.h
 *
 *  Created on: Dec 15, 2015
 *      Author: rdu
 */

#ifndef SRC_VISUALIZER_GRAPH_VIS_H_
#define SRC_VISUALIZER_GRAPH_VIS_H_

#include <vector>
#include <memory>

#include "opencv2/opencv.hpp"

#include "graph/graph.hpp"
#include "geometry/square_grid.hpp"

namespace librav {

namespace FastPlot
{
	// graph visualization
	void AddGraphLayer(std::shared_ptr<Graph_t<SquareCell*>> graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id = false);

	template<typename StateType>
	void AddGraphPathLayer(const Path_t<StateType>& path, cv::InputArray _src, cv::OutputArray _dst, cv::Scalar line_color = cv::Scalar( 255, 153, 51 ));
};

}

#endif /* SRC_VISUALIZER_GRAPH_VIS_H_ */
