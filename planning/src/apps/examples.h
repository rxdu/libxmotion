/*
 * examples.h
 *
 *  Created on: Mar 27, 2016
 *      Author: rdu
 */

#ifndef SRC_APPS_EXAMPLES_H_
#define SRC_APPS_EXAMPLES_H_

//#include

namespace srcl_ctrl {

class Examples{
public:
	Examples();
	~Examples();

public:
	void CreateQuadTreeFromImage();
	void CreateSquareGridFromImage();

	void CreateSquareGrid();
};

}



#endif /* SRC_APPS_EXAMPLES_H_ */
