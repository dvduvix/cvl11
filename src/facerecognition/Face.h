/*
 * Face.h
 *
 *  Created on: Oct 18, 2011
 *      Author: Vasilevski Dushan
 */

#ifndef FACE_H_
#define FACE_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "FaceProp.h"

using namespace std;
using namespace cv;

class Face
{
public:
	Face();
	virtual ~Face();

	int init(cv::Mat &frame);

	bool detectFace(cv::Mat &frame);

	string faceCascadeName;

	FaceProp faceProp;

	enum FaceMotion {
		U, D, L, R, UL, UR, DL, DR
	};

	FaceMotion direction;

private:
	CascadeClassifier faceCascade;

	Rect ROI;
	int factor;
	Vec3f plane;
};

#endif /* FACE_H_ */
