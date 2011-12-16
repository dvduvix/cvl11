/*
 * World.h
 *
 *  Created on: Dec 13, 2011
 *      Author: Vasilevski Dushan
 */

#ifndef WORLD_H_
#define WORLD_H_

#include <opencv2/imgproc/imgproc.hpp>

#include "FaceProp.h"

using namespace std;
using namespace cv;

class World {
public:
	World();
	virtual ~World();

	Vec3f get3DPoint(Point2i p, Mat &depthImage, float focus);
	CvMat *constructRotationMatrix(float roll, float pitch, float yaw);
	Vec3f normalFrom3DPoints(Vec3f s1, Vec3f s2, Vec3f s3);
	Vec3f normalFromArea(FaceProp prop, Mat &depthImage, float focus);
};

#endif /* WORLD_H_ */