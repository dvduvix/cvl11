/*
 * World.cpp
 *
 *  Created on: Dec 13, 2011
 *      Author: Vasilevski Dushan
 */

#include "World.h"

World::World() { }

World::~World() { }

Point3d World::get3DPoint(Point2i p, Mat &depthImage)
{
	printf("1 ... %d, %d\n", p.x, p.y);
	Point3d point;

	point.x = p.x;
	point.y = p.y;
	point.z = depthImage.at<float>(p.x, p.y);

	printf("2 ... %f \n", point.z);
	return point;
}

CvMat *World::constructRotationMatrix(float roll, float pitch, float yaw)
{
	CvMat *rotMat  = cvCreateMat(3, 3, CV_32FC1);

	cvmSet(rotMat, 0, 0, cos(yaw) * cos(pitch));
	cvmSet(rotMat, 0, 1, cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll));
	cvmSet(rotMat, 0, 2, cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll));

	cvmSet(rotMat, 1, 0, sin(yaw) * cos(pitch));
	cvmSet(rotMat, 1, 1, sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll));
	cvmSet(rotMat, 1, 2, sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll));

	cvmSet(rotMat, 2, 0, -sin(pitch));
	cvmSet(rotMat, 2, 1, cos(pitch) * sin(roll));
	cvmSet(rotMat, 2, 2, cos(pitch) * cos(roll));

	return rotMat;
}

Vec3f World::normalFromPoints(Point3f s1, Point3f s2, Point3f s3)
{
	Vec3f a, b, normal;

	a[0] = s1.x - s3.x;
	a[1] = s1.y - s3.y;
	a[2] = s1.z - s3.z;

	b[0] = s2.x - s3.x;
	b[1] = s2.y - s3.y;
	b[2] = s2.z - s3.z;

	normal[0] = a[1] * b[2] - a[2] * b[1];
	normal[1] = a[2] * b[0] - a[0] * b[2];
	normal[2] = a[0] * b[1] - a[1] * b[0];

	return normal;
}
