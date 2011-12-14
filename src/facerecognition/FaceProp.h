/*
 * FaceProp.h
 *
 *  Created on: Dec 13, 2011
 *      Author: Vasilevski Dushan
 */

#ifndef FACEPROP_H_
#define FACEPROP_H_

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class FaceProp {
public:
	FaceProp();
	virtual ~FaceProp();

	Point2i p1, p2, p3;
};

#endif /* FACEPROP_H_ */
