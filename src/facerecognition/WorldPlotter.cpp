#include "WorldPlotter.h"

WorldPlotter::WorldPlotter() {
  plot_size_x = 800;
  plot_size_y = 600;

  real_size_x = 4;
  real_size_y = 4;

  x_color      = Scalar(128, 128, 128);
  y_color      = Scalar(128, 128, 128);
  normal_color = Scalar(  0,   0, 255);
  object_color = Scalar(128, 128,   0);
  quad_color   = Scalar(  0, 128, 128);
  text_color   = Scalar(255, 255, 255);

  normal_thickness = 2;
  object_size      = 4;
  object_thickness = 3;
}

WorldPlotter::~WorldPlotter() {}

/**
 * This is the function that plots the helicopter AND the object of interest, along with its normal.
 */
void WorldPlotter::plotTopView(Point3f objectPosition, Point3f objectNormal,
                               Point3f quadPosition, Point3f quadOrientation) {

  Mat plot = Mat::zeros(plot_size_y, plot_size_x, CV_8UC3);

  line(plot, cvPoint(plot_size_x / 2, 0),
       cvPoint(plot_size_x / 2, plot_size_y), x_color);

  line(plot, cvPoint(0, plot_size_y / 2),
       cvPoint(plot_size_x, plot_size_y / 2), y_color);

  // Plot Normal Vector
  Point2i object_normal_p1, object_normal_p2;

  object_normal_p1.x = objectPosition.x / real_size_x * plot_size_x / 1000.0f
                     + plot_size_x / 2;
  object_normal_p1.y = objectPosition.y / real_size_y * plot_size_y / 1000.0f
                     + plot_size_y / 2;

  object_normal_p2.x = object_normal_p1.x - 25 * objectNormal.x;
  object_normal_p2.y = object_normal_p1.y - 25 * objectNormal.y;

  object_trace.push_back(object_normal_p1);
  plotTrace(plot, object_trace, object_color);

  line(plot, object_normal_p1, object_normal_p2, normal_color,
       normal_thickness);

  rectangle(plot,
            Point2i(object_normal_p1.x - object_size,
                    object_normal_p1.y - object_size),
            Point2i(object_normal_p1.x + object_size,
                    object_normal_p1.y + object_size),
            object_color, object_thickness);

  float x, y, z;
  x = quadPosition.x;
  y = quadPosition.y;
  z = quadPosition.z;

  float roll, pitch, yaw;
  roll  = quadOrientation.x;
  pitch = quadOrientation.y;
  yaw   = quadOrientation.z;

  float q_x, q_y;

  q_x = (x + 0.1 * cos(yaw)) / real_size_x * plot_size_x
      + plot_size_x / 2;
  q_y = (y + 0.1* sin(yaw)) / real_size_y * plot_size_y
      + plot_size_y / 2;

  x = x / real_size_x * plot_size_x + plot_size_x / 2;
  y = y / real_size_y * plot_size_y + plot_size_y / 2;

  quad_trace.push_back(Point2i(x, y));
  plotTrace(plot, quad_trace, quad_color);

  rectangle(plot, Point2i(x - object_size, y - object_size),
            Point2i(x + object_size, y + object_size), quad_color,
            object_thickness);

  rectangle(plot, Point2i(q_x - 1, q_y - 1), Point2i(q_x + 1, q_y + 1),
            quad_color, object_thickness);

  line(plot, Point2i(x, y), Point2i(q_x, q_y), normal_color, normal_thickness);

  putText(plot, "Object", Point2i(object_normal_p1.x, object_normal_p1.y - 10),
          FONT_HERSHEY_PLAIN, 1, text_color);

  putText(plot, "Quad", Point2i(x, y - 10), FONT_HERSHEY_PLAIN, 1, text_color);

  putText(plot, "iron curtain", Point2i(plot_size_x / 2 - 37, plot_size_y - 9),
          FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));

  objectPosition *= 0.001;

  Vec3f d = cv::Vec3f(objectPosition) - cv::Vec3f(quadPosition);
  Point3f distance = cv::Point3f(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]),
                                 0, 0);

  Point3f new_yaw = cv::Point3f(0, 0, atan2(objectPosition.y - quadPosition.y,
                                            objectPosition.x - quadPosition.x));

  cv::Vector<Point3f> coordinates;
  vector<string> labels;

  coordinates.push_back(objectPosition);
  labels.push_back("P.Object pos.");
  coordinates.push_back(quadPosition);
  labels.push_back("P.Quad pos.");
  coordinates.push_back(quadOrientation);
  labels.push_back("O.Quad orient.");
  coordinates.push_back(distance);
  labels.push_back("D.Distance");
  coordinates.push_back(new_yaw);
  labels.push_back("O.Goal Yaw");

  plotCoordinates(plot, coordinates, labels);

  namedWindow("Top View Plot");
  imshow("Top View Plot", plot);
}

void WorldPlotter::plotCoordinates(Mat &plot, Vector<Point3f> &coordinates,
                                   vector<string> &labels) {
  int count     = coordinates.size();
  int precision = 2;
  int width     = 10;
  String x = "x",
         y = "y",
         z = "z";
  String label;

  for(int i = 0; i < count; ++i) {
    label = &labels.at(i)[2];

    stringstream sstr;

    sstr << left << label.c_str() << right;

    if (labels.at(i)[0] == 'D') {
      x = "D";
      y = " ";
      z = " ";
    } else if (labels.at(i)[0] == 'O') {
      x = "r";
      y = "p";
      z = "y";
    }

    putText(plot, sstr.str(), Point2i(10, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
            text_color);

    stringstream sstrx;
    sstrx.precision(precision);
    sstrx.setf(ios::fixed, ios::floatfield);

    sstrx << "> " << x << ": ";
    sstrx.width(width);
    sstrx << right << coordinates[i].x;

    putText(plot, sstrx.str(), Point2i(120, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
            text_color);

    if (labels.at(i)[0] == 'D')
      continue;

    stringstream sstry;
    sstry.precision(precision);
    sstry.setf(ios::fixed, ios::floatfield);

    sstry << " " << y << ": ";
    sstry.width(width);
    sstry << right << coordinates[i].y;

    putText(plot, sstry.str(), Point2i(280, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
            text_color);

    stringstream sstrz;
    sstrz.precision(precision);
    sstrz.setf(ios::fixed, ios::floatfield);

    sstrz << " " << z << ": ";
    sstrz.width(width);
    sstrz << right << coordinates[i].z;

    putText(plot, sstrz.str(), Point2i(430, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
            text_color);
  }
}


void WorldPlotter::plotTrace(Mat &plot, vector<Point2i> &coordinates,
                             Scalar color) {
  for (unsigned int i = 0; i < coordinates.size() - 1; ++i) {
    line(plot, coordinates.at(i), coordinates.at(i + 1), color);
  }
}
