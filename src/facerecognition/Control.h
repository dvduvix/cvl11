/*
 * Control.h
 *
 *  Created on: Dec 14, 2011
 *      Author: Vasilevki Dushan
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <opencv2/imgproc/imgproc.hpp>
#include "mavconn.h"

using namespace std;
using namespace cv;

class Control {
public:
	Control();
	virtual ~Control();

	int flyToPos(Point3d pos, lcm_t *lcm, int compid);
};

#endif /* CONTROL_H_ */
