/*
 * World.cpp
 *
 *  Created on: Dec 13, 2011
 *      Author: Vasilevski Dushan
 */

#include "World.h"
#include <iostream>

World::World() { }

World::~World() { }

Vec3f World::get3DPoint(Point2i p, Mat &depthImage, float focus)
{
	Vec3f point;
	float z;
	int d = 1;

	z = depthImage.at<float>(p.y, p.x);

	while (z == 0) {
		vector<float> v;
		for (int i = -d; i <= d; ++i) {
			for (int j = -d; j <= d; ++j) {
				if (p.y + i > depthImage.rows - 1 || p.x + j > depthImage.cols - 1 ||
					p.y + i < 0 || p.x + j < 0)
					continue;

				if (i != d && i != -d && j != d && j != -d)
					continue;

				float z_t = depthImage.at<float>(p.y + i, p.x + j);

				if (z_t != 0) {
					int s_t = v.size();

					if (s_t == 0)
						v.push_back(z_t);
					else
					for (int s = 0; s < s_t; ++s) {
						if (z_t < v.at(s)) {
							vector<float>::iterator it = v.begin();
							it += s;
							v.insert(it, z_t);
						} else if (s == s_t - 1) {
							v.push_back(z_t);
						}
					}
				}
			}
		}

		if (v.size() == 0) {
			z = 0;
			d++;
		} else {
			z = v.at(floor((v.size() - 1) / 2.));
			break;
		}
	};

	point[0] = (p.x - depthImage.cols / 2.) * z / focus;
	point[1] = (p.y - depthImage.rows / 2.) * z / focus;
	point[2] = z;

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

Vec3f World::normalFrom3DPoints(Vec3f s1, Vec3f s2, Vec3f s3)
{
	Vec3f a, b, normal;

	a[0] = s1[0] - s3[0];
	a[1] = s1[1] - s3[1];
	a[2] = s1[2] - s3[2];

	b[0] = s2[0] - s3[0];
	b[1] = s2[1] - s3[1];
	b[2] = s2[2] - s3[2];

	float e0 = a[1] * b[2] - a[2] * b[1];
	float e1 = a[2] * b[0] - a[0] * b[2];
	float e2 = a[0] * b[1] - a[1] * b[0];

	float d = pow((float)(e0 * e0 + e1 * e1 + e2 * e2), (float)0.5);

	normal[0] = e0 / d;
	normal[1] = e1 / d;
	normal[2] = e2 / d;

	return normal;
}

Vec3f World::normalFromArea(FaceProp prop, Mat &depthImage, float focus)
{
	Vec3f normal;
	Rect area = prop.r;
	int f = 1,
		x = area.width,
		y = area.height,
		N = x * y / f;

	CvMat *res = cvCreateMat(3, 1, CV_32FC1);
	CvMat *matX = cvCreateMat(N, 3, CV_32FC1);
	CvMat *matZ = cvCreateMat(N, 1, CV_32FC1);

	for(int i = 0; i < x; i+=f) {
		for(int j = 0; j < y; j+=f) {
			Point2i p;
			p.x = area.x + i;
			p.y = area.y + j;

			Vec3f t_p = get3DPoint(p, depthImage, focus);

			int n = (i * x + j) / f;

			cvmSet(matX, n, 0, t_p[0]);
			cvmSet(matX, n, 1, t_p[1]);
			cvmSet(matX, n, 2, 1);
			cvmSet(matZ, n, 0, t_p[2]);
		}
	}


	cvSolve(matX, matZ, res, CV_SVD);

	float A = cvmGet(res, 0, 0);
	float B = cvmGet(res, 1, 0);
	float C = cvmGet(res, 2, 0);

	Vec3f t_p1 = get3DPoint(prop.p1, depthImage, focus);
	Vec3f t_p2 = get3DPoint(prop.p2, depthImage, focus);
	Vec3f t_p3 = get3DPoint(prop.p3, depthImage, focus);

	t_p1[2] =  (A * t_p1[0] + B * t_p1[1]) / C;
	t_p2[2] = (A * t_p2[0] + B * t_p2[1]) / C;
	t_p3[2] = (A * t_p3[0] + B * t_p3[1]) / C;

	normal = normalFrom3DPoints(t_p1, t_p2, t_p3);

	return normal;
}