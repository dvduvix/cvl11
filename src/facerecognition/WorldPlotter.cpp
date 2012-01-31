#include "WorldPlotter.h"

WorldPlotter::WorldPlotter() {
  plot_size_x = 800;
  plot_size_y = 600;

  real_size_x = 8;
  real_size_y = 6;

  x_color      = Scalar(128, 128, 128);
  y_color      = Scalar(128, 128, 128);
  normal_color = Scalar(  0,   0, 255);
  object_color = Scalar(128, 128,   0);
  quad_color   = Scalar(  0, 128, 128);
  text_color   = Scalar(255, 255, 255);

  normal_thickness = 2;
  object_size      = 4;
  object_thickness = 3;
  font_scale       = 0.7;
  marker_size      = 5;
}

WorldPlotter::~WorldPlotter() {}

/**
 * This is the function that plots the helicopter AND the object of interest,
 * along with its normal.
 *
 * @param objectPosition Position of an object
 * @param objectNormal Normal of an object
 * @param quadPosition Position of the quad
 * @param quadOrientation Roll, Pitch, and Yaw of the quad.
 */
void WorldPlotter::plotTopView(Point3f objectPosition, Point3f objectNormal,
                               Point3f quadPosition, Point3f quadOrientation) {

  Mat plot = Mat::zeros(plot_size_y, plot_size_x, CV_8UC3);

  plotAxes(plot);

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

  Point3f new_yaw = cv::Point3f(0, 0, arcTan(objectNormal.x,
                                             objectNormal.y));

  float normalization = sqrt(objectNormal.x * objectNormal.x +
                             objectNormal.y * objectNormal.y);
  float scale = -1.5;
  float golden_x = objectPosition.x + scale * objectNormal.x / normalization;
  float golden_y = objectPosition.y + scale * objectNormal.y / normalization;

  //printf("%f   %f \n", golden_x, golden_y);

  golden_x = golden_x / real_size_x * plot_size_x + plot_size_x / 2;
  golden_y = golden_y / real_size_y * plot_size_y + plot_size_y / 2;

  cv::Point2i goldenPoint = cv::Point2i(golden_x, golden_y);
  cv::circle(plot, goldenPoint, 2, Scalar(0, 0, 255), 2);

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
  labels.push_back("P.Goal Yaw");

  plotCoordinates(NULL, coordinates, labels);
  outputPlot = plot;
  namedWindow("Top View Plot");
  imshow("Top View Plot", plot);
}

/**
 * Outputs the coordinates given
 *
 * @param plot A pointer to the image. If set to NULL in outputs the
 *             coordinates in seperate window
 * @param coordinates Values of the coordinates
 * @param labels Labels of the coordiantes shown
 */
void WorldPlotter::plotCoordinates(Mat *plot, Vector<Point3f> &coordinates,
                                   vector<string> &labels) {
  int count = coordinates.size();
  Mat img;
  bool toPlot = false;

  if (plot == NULL) {
    toPlot = true;
    img = Mat::zeros(count * 15 + 10, plot_size_x, CV_8UC3);
    plot = &img;
  }

  int precision = 2;
  int width     = 10;

  String x, y, z;
  String label;

  for(int i = 0; i < count; ++i) {
    label = &labels.at(i)[2];

    stringstream sstr;

    float step;

    sstr << left << label.c_str() << right;

    if (labels.at(i)[0] == 'D') {
      x = "D";
      y = " ";
      z = " ";
      step = (float)(plot_size_x - 120) / 1.0f;
    } else if (labels.at(i)[0] == 'O') {
      x = "r";
      y = "p";
      z = "y";
      step = (float)(plot_size_x - 120) / 3.0f;
    } else {
      x = "x";
      y = "y";
      z = "z";
      step = (float)(plot_size_x - 120) / 3.0f;
    }

    putText(*plot, sstr.str(), Point2i(10, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
            text_color);

    stringstream sstrx;
    sstrx.precision(precision);
    sstrx.setf(ios::fixed, ios::floatfield);

    sstrx << "> " << x << ": ";
    sstrx.width(width);
    sstrx << right << coordinates[i].x;

    putText(*plot, sstrx.str(), Point2i(120, 15 * (i + 1)),
            FONT_HERSHEY_PLAIN, 1, text_color);

    if (labels.at(i)[0] == 'D')
      continue;

    stringstream sstry;
    sstry.precision(precision);
    sstry.setf(ios::fixed, ios::floatfield);

    sstry << " " << y << ": ";
    sstry.width(width);
    sstry << right << coordinates[i].y;

    putText(*plot, sstry.str(), Point2i(120 + step, 15 * (i + 1)),
            FONT_HERSHEY_PLAIN, 1, text_color);

    stringstream sstrz;
    sstrz.precision(precision);
    sstrz.setf(ios::fixed, ios::floatfield);

    sstrz << " " << z << ": ";
    sstrz.width(width);
    sstrz << right << coordinates[i].z;

    putText(*plot, sstrz.str(), Point2i(120 + 2 * step, 15 * (i + 1)),
            FONT_HERSHEY_PLAIN, 1, text_color);
  }

  if (toPlot) {
    namedWindow("Coordinates");
    imshow("Coordinates", *plot);
  }
}

/**
 * Traces the motion of the quad and the object
 *
 * @param plot A pointer to the image. If set to NULL in outputs the
 *             coordinates in seperate window
 * @param coordinates Values of the coordinates
 * @param color Color of the trace
 */
void WorldPlotter::plotTrace(Mat &plot, vector<Point2i> &coordinates,
                             Scalar color) {
  for (unsigned int i = 0; i < coordinates.size() - 1; ++i) {
    line(plot, coordinates.at(i), coordinates.at(i + 1), color);
  }
}

/**
 * Plots the axes in the ploter
 *
 * @param plot A pointer to the image. If set to NULL in outputs the
 *             coordinates in seperate window
 */
void WorldPlotter::plotAxes(cv::Mat &plot) {
  int marker_x,
      marker_y;

  line(plot, cvPoint(plot_size_x / 2, 0),
       cvPoint(plot_size_x / 2, plot_size_y), x_color);
  line(plot, cvPoint(0, plot_size_y / 2),
       cvPoint(plot_size_x, plot_size_y / 2), y_color);

  for(int i = -real_size_x / 2; i < real_size_x / 2; ++i) {
    marker_x = i / real_size_x * plot_size_x + plot_size_x / 2;
    marker_y = plot_size_y / 2;

    line(plot, Point(marker_x, marker_y - marker_size),
         Point(marker_x, marker_y + marker_size), x_color);

    stringstream sstr;
    sstr << i;

    putText(plot, sstr.str(), Point2i(marker_x, marker_y - marker_size * 1.5),
            FONT_HERSHEY_PLAIN, font_scale, x_color);
  }

  for(int i = -real_size_y / 2; i < real_size_y / 2; ++i) {
    marker_y = i / real_size_y * plot_size_y + plot_size_y / 2;
    marker_x = plot_size_x / 2;

    line(plot, Point(marker_x, marker_y - marker_size),
         Point(marker_x, marker_y + marker_size), y_color);

    stringstream sstr;
    sstr << i;

    putText(plot, sstr.str(), Point2i(marker_x + marker_size * 1.5, marker_y),
            FONT_HERSHEY_PLAIN, font_scale, y_color);
  }
}

/**
 * Calculates the yaw fot the give X and Y
 *
 * @param x X coordinate of the vector
 * @param y Y coordinate of the vector
 */
float WorldPlotter::arcTan(float x, float y) {
  float angle;
  float ax = abs(x);
  float ay = abs(y);

  if (x > 0 && y > 0)
    angle = atan(ay / ax);
  else if (x < 0 && y > 0)
    angle = M_PI_2 + atan(ax / ay);
  else if (x < 0 && y < 0)
    angle = -M_PI_2 - atan(ax / ay);
  else if (x > 0 && y < 0)
    angle = -atan(ay / ax);
  else if (x == 0)
    angle = (y > 0) ? M_PI_2 : -M_PI_2;
  else if (y == 0)
    angle = (x > 0) ? 0 : M_PI;


  if (x == 0 && y == 0)
    angle = 0;

  return angle / M_PI * 180;
}
