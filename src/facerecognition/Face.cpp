/*
 * Face.cpp
 *
 *  Created on: Oct 18, 2011
 *      Author: Vasilevski Dushan
 */

#include "Face.h"

Face::Face() {}

Face::~Face() {}

/**
 * @brief Detect face charachtersitcs
 *
 */
bool Face::detectFace(cv::Mat &frame)
{
	Mat dFrame;
	int tr = 10;

	cv::resize(frame, dFrame, Size(), 1. / factor, 1. / factor, INTER_AREA);

	std::vector<Rect> faces;

	Mat imgROI = dFrame(ROI);

	faceCascade.detectMultiScale(imgROI, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

	if (faces.size() == 0)
		return false;

	Rect face    = faces[0];
	face.height *= factor;
	face.width  *= factor;
	face.x 		= (ROI.x + face.x) * factor;
	face.y 		= (ROI.y + face.y) * factor;

	int maxx = max(ROI.x + faces[0].x - dFrame.cols / 4, 1);
	int maxy = max(ROI.y + faces[0].y - dFrame.rows / 4, 1);
	int minx = min(faces[0].width + dFrame.cols / 2, dFrame.cols - maxx - 1);
	int miny = min(faces[0].height + dFrame.rows / 2, dFrame.rows - maxy - 1);

	ROI = cvRect(maxx, maxy, minx, miny);

	Point2i pp1, pp2;

	pp1.x = maxx * factor;
	pp1.y = maxy * factor;
	pp2.x = (maxx + minx) * factor;
	pp2.y = (maxy + miny) * factor;

	///////////////////////////////////////////////////////////////////////////
	// Picking points

	std::vector<Rect> eyes;
	Rect cFace = face;
	cFace.height /= 2.;

	eyesCascade.detectMultiScale(frame(cFace), eyes, 1.05, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(15, 15));

	Point p1, p2, p3;

	p1.x = 0.3 * face.width + face.x;
	p2.x = 0.7 * face.width + face.x;
	p1.y = face.height / 2.6 + face.y;
	p2.y = p1.y;

	for (int i = 0; i < eyes.size(); ++i) {
		if (i == 2)
			break;

		Rect eye = eyes[i];

		if (eye.x < faces[0].width / 2.) {
			p1.x = face.x + eye.width / 2. + eye.x;
			p1.y = face.y + eye.height / 2. + eye.y;
		} else {
			p2.x = face.x + eye.width / 2. + eye.x;
			p2.y = face.y + eye.height / 2. + eye.y;
		}
	}

	std::vector<Rect> lips;
	cFace.y += cFace.height;

	lipsCascade.detectMultiScale(frame(cFace), lips, 1.05, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(25, 15));

	if (lips.size() == 1) {
		p3.x = face.x + lips[0].width / 2. + lips[0].x;
		p3.y = face.y + lips[0].height / 2. + lips[0].y + cFace.height;
	} else {
		p3.x = 0.5 * face.width + face.x;
		p3.y = 0.78 * face.height + face.y;
	}

	Vec2f c;
	c[0] = 0.5 * face.width + face.x;
	c[1] = 0.5 * face.height + face.y;

	Rect iFace;
	iFace.x = 0.3 * face.width + face.x;
	iFace.y = 0.3 * face.height + face.y;
	iFace.width = 0.4 * face.width;
	iFace.height = 0.4 * face.height;

	int dX = faceProp.p1.x + faceProp.p2.x + faceProp.p3.x - p1.x - p2.x - p3.x;
	int dY = faceProp.p1.y + faceProp.p2.y + faceProp.p3.y - p1.y - p2.y - p3.y;

	if (abs(abs(dX) - abs(dY)) > tr) {
		if (abs(dX) > abs(dY)) {
			if (dX > 0)
				direction = FaceMotion::L;
			else
				direction = FaceMotion::R;
		} else {
			if (dY > 0)
				direction = FaceMotion::U;
			else
				direction = FaceMotion::D;
		}
	} else {
		if (dX > 0 && dY > 0)
			direction = FaceMotion::UL;
		else if (dX < 0 && dY < 0)
			direction = FaceMotion::DR;
		else if (dX > 0 && dY < 0)
			direction = FaceMotion::DL;
		else
			direction = FaceMotion::UR;
	}

	faceProp.p1 = p1;
	faceProp.p2 = p2;
	faceProp.p3 = p3;
	faceProp.c = c;
	faceProp.r = iFace;

	return true;
}

int Face::init(cv::Mat &frame)
{
	if (!faceCascade.load(faceCascadeName)) {
		printf("--(!)Error loading %s\n", faceCascadeName.c_str());
		return -1;
	}

	if (!eyesCascade.load(eyesCascadeName)) {
		printf("--(!)Error loading %s\n", eyesCascadeName.c_str());
		return -1;
	}

	if (!lipsCascade.load(lipsCascadeName)) {
		printf("--(!)Error loading %s\n", lipsCascadeName.c_str());
		return -1;
	}

	factor = 2;

	ROI = cvRect(0, 0, frame.cols / factor, frame.rows / factor);

	return 0;
}


