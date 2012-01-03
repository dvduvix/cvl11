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

Vec3f World::get3DPoint(Vec2i p, Mat &depthImage, float focus)
{
	Vec3f point;
	float z;
	int d = 1;

	z = depthImage.at<float>(p[1], p[0]);

	while (z == 0) {
		vector<float> v;
		for (int i = -d; i <= d; ++i) {
			for (int j = -d; j <= d; ++j) {
				if (p[1] + i > depthImage.rows - 1 || p[0] + j > depthImage.cols - 1 ||
					p[1] + i < 0 || p[0] + j < 0)
					continue;

				if (i != d && i != -d && j != d && j != -d)
					continue;

				float z_t = depthImage.at<float>(p[1] + i, p[0] + j);

				if (z_t != 0)
  				v.push_back(z_t);
				
			}
		}

		if (v.size() == 0) {
			z = 0;
			d++;
		} else {
		  z = getMed(v);
		  
			//z = v.at(floor((v.size() - 1) / 2.));
			break;
		}
	};

	point[0] = (p[0] - depthImage.cols / 2.) * z / focus;
	point[1] = (p[1] - depthImage.rows / 2.) * z / focus;
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

	float d = sqrt(e0 * e0 + e1 * e1 + e2 * e2);

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

Vec3f World::globalPoint(const mavlink_message_t *msg, PxSHMImageClient *client,
		Vec2f &p, Mat &intresic, Mat &depthImage)
{
	float roll, pitch, yaw;
	float x, y, z;

	client->getRollPitchYaw(msg, roll, pitch, yaw);
	client->getGroundTruth(msg, x, y, z);

	float ca = cos(yaw);
	float sa = sin(yaw);
	float cb = cos(pitch);
	float sb = sin(pitch);
	float cg = cos(roll);
	float sg = sin(roll);

	float H1t[4][4] = {
			{ca * cb,  ca * sb * sg - sa * cg,  ca * sb * cg + sa * sg,  x * 1000},
			{sa * cb,  sa * sb * sg + ca * cg,  sa * sb * cg - ca * sg,  y * 1000},
			{-sb, 	   cb * sg, 				        cb * cg, 				         z * 1000},
			{0, 	     0, 					            0, 					             1	     }
	};


	float H2t[4][4] = {
			{0, 1, 0, 0},
			{0, 0, 1, 0},
			{1, 0, 0, 0},
			{0, 0, 0, 1}
	};

	Mat H1(4, 4, CV_32FC1, H1t);
	Mat H2(4, 4, CV_32FC1, H2t);

	Mat H = H1 * H2.inv();
	
	Vec3f ooo = get3DPoint(p, depthImage, intresic.at<float>(0, 0));
	
	cv::Vec4f cameraPoint = cv::Vec4f(ooo[0] * 1000.0f,
	    			                        ooo[1] * 1000.0f,
			                              ooo[2] * 1000.0f,
  			                            1);

	Mat X = H * cv::Mat(cameraPoint);

	Vec3f fP = Vec<float, 3>(X.at<float>(0, 0) / X.at<float>(3, 0),
	                         X.at<float>(1, 0) / X.at<float>(3, 0),
        			             X.at<float>(2, 0) / X.at<float>(3, 0));
	
	return fP;
}

void swap(vector<float>& t, int i , int j) {
    float a = t[i];
    t[i] = t[j];
    t[j] = a;
}

int partition(vector<float>& t, int p, int r){
    float x = t[r];
    int i = p-1;
    for(int j = p;j<=r-1;j++){

        if(t[j]<=x){
            i++;
            swap(t,i,j);
        }
    }
    swap(t,i+1,r);
    return i+1;
}

int randomizedPartition(vector<float>& t, int p, int r){
    int i = (rand()%(r-p+1))+p;
    swap(t,r,i);
    return partition(t,p,r);
}

float select(vector<float>& t,int p,int r,int i){
    while(true){
        if(p==r) return t[p];
        int q = randomizedPartition(t,p,r);
        int k = q-p+1;
        if(i==k) return t[q];
        else if(i<k)
            r=q-1;
        else{
            p=q+1;
            i-=k;
        }
    }

    return 0.0f;
}

float World::getMed(vector<float>& t) {
    return select(t,0,t.size()-1,(t.size()+1)/2);
}
