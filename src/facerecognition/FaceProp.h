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

/**
  * Class containing the fatial parameters
  */
class FaceProp {
public:
	FaceProp();
	virtual ~FaceProp();

	Vec2f p1, p2, p3;
	Vec2f c;
	Rect r;
};

#endif /* FACEPROP_H_ */
