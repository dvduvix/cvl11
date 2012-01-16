/*
 * WorldPlotter.h
 *
 *  Created on: Jan 5, 2012
 *      Author: Dushan Vasilevski
 *              Sinan Mohamed
 */

#ifndef WORLDPLOTTER_H_
#define WORLDPLOTTER_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class WorldPlotter {
 public:
  WorldPlotter();
  virtual ~WorldPlotter();

  void plotTopView(Point3f objectPosition, Point3f objectNormal,
                   Point3f quadPosition, Point3f quadOrientation);

  int plot_size_x;
  int plot_size_y;
  float real_size_x;
  float real_size_y;

  Scalar x_color;
  Scalar y_color;
  Scalar normal_color;
  Scalar object_color;
  Scalar quad_color;
  Scalar text_color;

  int normal_thickness;
  int object_size;
  int object_thickness;
  int marker_size;
  double font_scale;

 private:
  void plotCoordinates(Mat *plot, Vector<Point3f> &coordinates,
                       vector<string> &labels);
  void plotTrace(Mat &plot, vector<Point2i> &coordinates, Scalar color);
  void plotAxes(cv::Mat &plot);

  vector<Point2i> quad_trace;
  vector<Point2i> object_trace;
};

#endif /* WORLDPLOTTER_H_ */
