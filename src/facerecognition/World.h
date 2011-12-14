/*
 * World.h
 *
 *  Created on: Dec 13, 2011
 *      Author: Vasilevski Dushan
 */

#ifndef WORLD_H_
#define WORLD_H_

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class World {
public:
	World();
	virtual ~World();

	Point3d get3DPoint(Point2i p, Mat &depthImage);
	CvMat *constructRotationMatrix(float roll, float pitch, float yaw);
	Vec3f normalFromPoints(Point3f s1, Point3f s2, Point3f s3);
};

#endif /* WORLD_H_ */
