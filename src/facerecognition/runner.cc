/**
 * @file
 *   @brief Face Tracking
 *
 *   @author Vasilevski Dushan
 *
 */
#include <cstdio>
#include <unistd.h>
#include <glib.h>
#include "mavconn.h"
#include "Face.h"
#include "World.h"
#include "Control.h"
#include "StereoProc.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <signal.h>
#include <interface/shared_mem/PxSHMImageClient.h>

#include <sys/time.h>
#include <time.h>

bool verbose      = false;
bool gui          = false;
bool agui         = false;
bool ok           = false;
bool foundFace    = false;
bool isInit       = true;
bool quit         = false;
bool debug        = 0;
bool fpsb         = false;
double timef      = 0;
int imageCounter  = 0;
int sysid         = 42;
int compid        = 112;
static GString* configFile = g_string_new("config.cfg");

int countf, fps;
struct timeval tv;
string stereoCfg;
Face *face;
World *world;
Control *control;
StereoProc stereo;

std::string fileBaseName("frame");
std::string fileExt(".png");

Vec3f apt(100, 100, 100);

using namespace std;
using namespace cv;

float colormap_jet[128][3] = { { 0.0f, 0.0f, 0.53125f },
    { 0.0f, 0.0f, 0.5625f }, { 0.0f, 0.0f, 0.59375f }, { 0.0f, 0.0f, 0.625f }, {
        0.0f, 0.0f, 0.65625f }, { 0.0f, 0.0f, 0.6875f },
    { 0.0f, 0.0f, 0.71875f }, { 0.0f, 0.0f, 0.75f }, { 0.0f, 0.0f, 0.78125f }, {
        0.0f, 0.0f, 0.8125f }, { 0.0f, 0.0f, 0.84375f }, { 0.0f, 0.0f, 0.875f },
    { 0.0f, 0.0f, 0.90625f }, { 0.0f, 0.0f, 0.9375f }, { 0.0f, 0.0f, 0.96875f },
    { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.03125f, 1.0f }, { 0.0f, 0.0625f, 1.0f }, {
        0.0f, 0.09375f, 1.0f }, { 0.0f, 0.125f, 1.0f },
    { 0.0f, 0.15625f, 1.0f }, { 0.0f, 0.1875f, 1.0f }, { 0.0f, 0.21875f, 1.0f },
    { 0.0f, 0.25f, 1.0f }, { 0.0f, 0.28125f, 1.0f }, { 0.0f, 0.3125f, 1.0f }, {
        0.0f, 0.34375f, 1.0f }, { 0.0f, 0.375f, 1.0f },
    { 0.0f, 0.40625f, 1.0f }, { 0.0f, 0.4375f, 1.0f }, { 0.0f, 0.46875f, 1.0f },
    { 0.0f, 0.5f, 1.0f }, { 0.0f, 0.53125f, 1.0f }, { 0.0f, 0.5625f, 1.0f }, {
        0.0f, 0.59375f, 1.0f }, { 0.0f, 0.625f, 1.0f },
    { 0.0f, 0.65625f, 1.0f }, { 0.0f, 0.6875f, 1.0f }, { 0.0f, 0.71875f, 1.0f },
    { 0.0f, 0.75f, 1.0f }, { 0.0f, 0.78125f, 1.0f }, { 0.0f, 0.8125f, 1.0f }, {
        0.0f, 0.84375f, 1.0f }, { 0.0f, 0.875f, 1.0f },
    { 0.0f, 0.90625f, 1.0f }, { 0.0f, 0.9375f, 1.0f }, { 0.0f, 0.96875f, 1.0f },
    { 0.0f, 1.0f, 1.0f }, { 0.03125f, 1.0f, 0.96875f },
    { 0.0625f, 1.0f, 0.9375f }, { 0.09375f, 1.0f, 0.90625f }, { 0.125f, 1.0f,
        0.875f }, { 0.15625f, 1.0f, 0.84375f }, { 0.1875f, 1.0f, 0.8125f }, {
        0.21875f, 1.0f, 0.78125f }, { 0.25f, 1.0f, 0.75f }, { 0.28125f, 1.0f,
        0.71875f }, { 0.3125f, 1.0f, 0.6875f }, { 0.34375f, 1.0f, 0.65625f }, {
        0.375f, 1.0f, 0.625f }, { 0.40625f, 1.0f, 0.59375f }, { 0.4375f, 1.0f,
        0.5625f }, { 0.46875f, 1.0f, 0.53125f }, { 0.5f, 1.0f, 0.5f }, {
        0.53125f, 1.0f, 0.46875f }, { 0.5625f, 1.0f, 0.4375f }, { 0.59375f,
        1.0f, 0.40625f }, { 0.625f, 1.0f, 0.375f },
    { 0.65625f, 1.0f, 0.34375f }, { 0.6875f, 1.0f, 0.3125f }, { 0.71875f, 1.0f,
        0.28125f }, { 0.75f, 1.0f, 0.25f }, { 0.78125f, 1.0f, 0.21875f }, {
        0.8125f, 1.0f, 0.1875f }, { 0.84375f, 1.0f, 0.15625f }, { 0.875f, 1.0f,
        0.125f }, { 0.90625f, 1.0f, 0.09375f }, { 0.9375f, 1.0f, 0.0625f }, {
        0.96875f, 1.0f, 0.03125f }, { 1.0f, 1.0f, 0.0f },
    { 1.0f, 0.96875f, 0.0f }, { 1.0f, 0.9375f, 0.0f }, { 1.0f, 0.90625f, 0.0f },
    { 1.0f, 0.875f, 0.0f }, { 1.0f, 0.84375f, 0.0f }, { 1.0f, 0.8125f, 0.0f }, {
        1.0f, 0.78125f, 0.0f }, { 1.0f, 0.75f, 0.0f }, { 1.0f, 0.71875f, 0.0f },
    { 1.0f, 0.6875f, 0.0f }, { 1.0f, 0.65625f, 0.0f }, { 1.0f, 0.625f, 0.0f }, {
        1.0f, 0.59375f, 0.0f }, { 1.0f, 0.5625f, 0.0f },
    { 1.0f, 0.53125f, 0.0f }, { 1.0f, 0.5f, 0.0f }, { 1.0f, 0.46875f, 0.0f }, {
        1.0f, 0.4375f, 0.0f }, { 1.0f, 0.40625f, 0.0f }, { 1.0f, 0.375f, 0.0f },
    { 1.0f, 0.34375f, 0.0f }, { 1.0f, 0.3125f, 0.0f }, { 1.0f, 0.28125f, 0.0f },
    { 1.0f, 0.25f, 0.0f }, { 1.0f, 0.21875f, 0.0f }, { 1.0f, 0.1875f, 0.0f }, {
        1.0f, 0.15625f, 0.0f }, { 1.0f, 0.125f, 0.0f },
    { 1.0f, 0.09375f, 0.0f }, { 1.0f, 0.0625f, 0.0f }, { 1.0f, 0.03125f, 0.0f },
    { 1.0f, 0.0f, 0.0f }, { 0.96875f, 0.0f, 0.0f }, { 0.9375f, 0.0f, 0.0f }, {
        0.90625f, 0.0f, 0.0f }, { 0.875f, 0.0f, 0.0f },
    { 0.84375f, 0.0f, 0.0f }, { 0.8125f, 0.0f, 0.0f }, { 0.78125f, 0.0f, 0.0f },
    { 0.75f, 0.0f, 0.0f }, { 0.71875f, 0.0f, 0.0f }, { 0.6875f, 0.0f, 0.0f }, {
        0.65625f, 0.0f, 0.0f }, { 0.625f, 0.0f, 0.0f },
    { 0.59375f, 0.0f, 0.0f }, { 0.5625f, 0.0f, 0.0f }, { 0.53125f, 0.0f, 0.0f },
    { 0.5f, 0.0f, 0.0f } };

struct united {
  PxSHMImageClient *imageClient;
  lcm_t *lcm;
};

void colorDepthImage(cv::Mat& imgDepth, cv::Mat& imgColoredDepth) {
  imgColoredDepth = cv::Mat::zeros(imgDepth.size(), CV_8UC3);

  for (int i = 0; i < imgColoredDepth.rows; ++i) {
    const float *depth = imgDepth.ptr<float>(i);

    unsigned char *pixel = imgColoredDepth.ptr<unsigned char>(i);

    for (int j = 0; j < imgColoredDepth.cols; ++j) {
      if (depth[j] != 0) {
        int idx = fmin(depth[j], 10.0f) / 10.0f * 127.0f;
        idx = 127 - idx;

        pixel[0] = colormap_jet[idx][2] * 255.0f;
        pixel[1] = colormap_jet[idx][1] * 255.0f;
        pixel[2] = colormap_jet[idx][0] * 255.0f;
      }

      pixel += 3;
    }
  }
}

void signalHandler(int signal) {
  if (signal == SIGINT) {
    fprintf(stderr, "# INFO: Quitting...\n");
    quit = true;
    exit(EXIT_SUCCESS);
  }
}

///////////////////////////////////////////////////////////////////////////////
// PRAISE KALMAN
//////////////////////////////////////////////////////////////////////////////
KalmanFilter KF(2, 2, 0);
Mat_<float> measurement(2, 1);
Point gW(0, 0);

void imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
                  const mavconn_mavlink_msg_container_t* container,
                  void* user) {
  ///////////////////////////////////////////////////////////////////////////
  Mat prediction = KF.predict();
  Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
  ///////////////////////////////////////////////////////////////////////////

  struct timeval t1, t2;
  if (gui || fpsb)
    gettimeofday(&t1, NULL);

  const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
  struct united *clientHandler = static_cast<struct united *>(user);

  PxSHMImageClient* client = clientHandler->imageClient;
  lcm_t *lcm = clientHandler->lcm;

  cv::Mat imgToSave;
  cv::Mat imgL, imgR, imgDepth, imgRectified;

  bool s = stereo.init(stereoCfg);

  if (!s) {
    printf("ERROR: Stereo not init-ed\n");
    exit(EXIT_FAILURE);
  }

  Mat intrinsicMat;
  Mat imgDepthColor;

  stereo.getImageInfo(intrinsicMat);
  float focus = intrinsicMat.at<float>(0, 0);

  if (client->readStereoImage(msg, imgL, imgR)) {
    if (isInit) {
      face->init(imgL);
      isInit = false;
    }

    foundFace = face->detectFace(imgL);

    if (foundFace) {
      stereo.process(imgL, imgR, imgRectified, imgDepth);

      if (agui) {
        cv::Mat *imgDepthCopy = new cv::Mat();
        imgDepth.copyTo(*imgDepthCopy);

        colorDepthImage(imgDepth, imgDepthColor);
        cv::namedWindow("Depth map");
        cv::imshow("Depth map", imgDepthColor);
      }

      IplImage iImgL = imgL;

      cvLine(&iImgL, face->faceProp.p1, face->faceProp.p2,
             cvScalar(255, 0, 0, 1), 2);
      cvLine(&iImgL, face->faceProp.p1, face->faceProp.p3,
          cvScalar(255, 0, 0, 1));
      cvLine(&iImgL, face->faceProp.p2, face->faceProp.p3,
          cvScalar(255, 0, 0, 1), 2);

      Vec3f p3d3 = world->get3DPoint(face->faceProp.p3, imgDepth, focus);
      Vec3f normal = world->normalFrom3DPoints(
          world->get3DPoint(face->faceProp.p1, imgDepth, focus),
          world->get3DPoint(face->faceProp.p2, imgDepth, focus), p3d3);

      Vec3f pp = world->globalPoint(msg, client, face->faceProp.c, intrinsicMat,
          imgDepth);

     /* if (verbose)
        std::cerr << pp[0] << " " << pp[1] << " " << pp[2] << "\n";
*/
//			Point2i p;
//			p.x = face->faceProp.c[0] - 50 * normal[0];
//			p.y = face->faceProp.c[1] - 50 * normal[1];
//
//			cvLine(&iImgL, cvPoint(face->faceProp.c[0], face->faceProp.c[1]),
//					p, cvScalar(0, 0, 0, 1), 3);

        Point2i pw, kw;
        pw.x = imgL.cols / 2. - imgL.cols / 4. * normal[0];
        pw.y = imgL.rows - 8;

        if (gui)
          cvLine(&iImgL, cvPoint(imgL.cols / 2, pw.y), pw, cvScalar(255, 0, 0, 1),
                 10);

        measurement(0) = pw.x * (1 - abs(pw.x - gW.x) / imgL.cols);
        measurement(1) = pw.y;

        Mat estimated = KF.correct(measurement);
        Point statePt(estimated.at<float>(0), estimated.at<float>(1));

        gW = kw = statePt;

        if (gui)
          cvLine(&iImgL, cvPoint(imgL.cols / 2, kw.y - 8), cvPoint(kw.x, kw.y - 8),
                 cvScalar(128, 0, 0, 1), 10);
    } else {
    }

    if (ok) {
      //control->keepDistance(msg, client, p3d3, lcm, compid);
      if (verbose)
        printf("Some morron decided to let me fly... \n");

      float x, y, z;
      client->getGroundTruth(msg, x, y, z);
      if (verbose)
        printf("Location : x: %f y: %f z: %f\n", x, y, z);

      float er = 0.001;
      Vec3f ap(0, 0, 0);
      float rate = M_PI / 16;


      if (apt[0] == 100 || (abs(apt[0] - x) < er && abs(apt[1] - y) < er &&
                            abs(apt[2] - z) < er)) {
        apt = control->loopAround(msg, client, ap, rate, lcm, compid);
        if (verbose)
          printf("Sending : x: %f y: %f z: %f\n", apt[0], apt[1], apt[1]);
      }

      control->flyToPos(apt, 0, lcm, compid);

    }


    if (gui || fpsb) {
      gettimeofday(&t2, NULL);
      uint64_t time = ((uint64_t) t2.tv_sec) * 1000000 + t2.tv_usec
                    - ((uint64_t) t1.tv_sec) * 1000000 - t1.tv_usec;

      stringstream ss(stringstream::in | stringstream::out);

      timef += time / 1000000.;
      countf++;

      if (timef > 1.0) {
        fps = countf;
        timef = 0;
        countf = 0;
      }

      ss << fps;

      if (fpsb)
        cout << "FPS: " << fps << endl;

      putText(imgL, ss.str(), cvPoint(10, 20), FONT_HERSHEY_PLAIN, 1,
          cvScalar(255, 0, 0, 1));
    }

    if (gui) {
      if ((client->getCameraConfig() & PxSHM::CAMERA_FORWARD_LEFT)
          == PxSHM::CAMERA_FORWARD_LEFT) {
        cv::namedWindow("Left Image (Forward Camera)");
        cv::imshow("Left Image (Forward Camera)", imgL);
      } else {
        cv::namedWindow("Left Image (Downward Camera)");
        cv::imshow("Left Image (Downward Camera)", imgL);
      }
    }
  }

  if (gui || agui) {
    int c = cv::waitKey(3);

    switch (static_cast<char>(c)) {
      case 'f': {
        char index[20];
        sprintf(index, "%04d", imageCounter++);
        cv::imwrite(std::string(fileBaseName + index + fileExt).c_str(),
            imgToSave);
      }
      break;
      default:
      break;
    }
  }
}

static void mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel,
                            const mavconn_mavlink_msg_container_t* container,
                            void * user) {
  const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
//  printf("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n", msg->msgid, channel, msg->sysid, msg->compid);

  switch (msg->msgid) {

    uint32_t receiveTime;
    uint32_t sendTime;

  case MAVLINK_MSG_ID_ATTITUDE:
    gettimeofday(&tv, NULL);
    receiveTime = tv.tv_usec;
    sendTime = mavlink_msg_attitude_get_time_boot_ms(msg);
    //printf("Received attitude message, transport took %f ms\n", (receiveTime - sendTime)/1000.0f);
  break;
  case MAVLINK_MSG_ID_GPS_RAW_INT: {
    mavlink_gps_raw_int_t gps;
    mavlink_msg_gps_raw_int_decode(msg, &gps);
    //printf("GPS: lat: %f, lon: %f, alt: %f\n", gps.lat/(double)1E7, gps.lon/(double)1E7, gps.alt/(double)1E6);
    break;
  }
  case MAVLINK_MSG_ID_RAW_PRESSURE: {
    mavlink_raw_pressure_t p;
    mavlink_msg_raw_pressure_decode(msg, &p);
    //printf("PRES: %f\n", p.press_abs/(double)1000);
  }
  break;
  default:
    //printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
  break;
  }
}

void *lcm_wait(void *lcm_ptr) {
  lcm_t *lcm = (lcm_t *) lcm_ptr;

  while (1) {
    lcm_handle(lcm);
  }

  return NULL;
}

static GOptionEntry entries[] = {
    //{ "sysid", 'a', 0, G_OPTION_ARG_INT, &sysid, "ID of this system, 1-255", "42"},
//{ "compid", 'c', 0, G_OPTION_ARG_INT, &compid, "ID of this component, 1-255", "55" },
    { "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose",
        (verbose) ? "true" : "false" }, { "gui", 'g', 0, G_OPTION_ARG_NONE,
        &gui, "Show windows", (gui) ? "true" : "false" }, { "agui", 'a', 0,
        G_OPTION_ARG_NONE, &agui, "Show advanced windows",
            (agui) ? "true" : "false" }, { "fps", 'f', 0, G_OPTION_ARG_NONE,
        &fpsb, "Show advanced windows", (fpsb) ? "true" : "false" }, { "oktogo",
        'o', 0, G_OPTION_ARG_NONE, &ok, "Ok to go", (ok) ? "true" : "false" },
        //{ "config", 'f', 0, G_OPTION_ARG_STRING, configFile, "Filename of paramClient config file", "config/parameters_2pt.cfg"},
    { NULL }
};

int main(int argc, char* argv[]) {
  /////////////////////////////////////////////////////////////////////////////
    ifstream myfile(configFile->str);

    if (myfile.is_open()) {
      getline(myfile, stereoCfg);
      myfile.close();
    } else {
      printf("Unable to open conf file.");
      exit(0);
    }

    face = new Face();

    face->faceCascadeName = "haarcascade_frontalface_alt.xml";
    face->eyesCascadeName = "haarcascade_eye_tree_eyeglasses.xml";
    face->lipsCascadeName = "haarcascade_mcs_mouth.xml";
  /////////////////////////////////////////////////////////////////////////////

  GError *error = NULL;
  GOptionContext *context;

  context = g_option_context_new("- localize based on natural features");
  g_option_context_add_main_entries(context, entries, "Localization");

  if (!g_option_context_parse(context, &argc, &argv, &error)) {
    g_print("Option parsing failed: %s\n", error->message);
    exit(EXIT_FAILURE);
  }

  g_option_context_free(context);

  printf("Verbose : %d, Gui : %d, AGui : % d, Ok : %d \n", verbose, gui, agui, ok);

  struct united clientHandler;
  lcm_t* lcm = lcm_create("udpm://");

  clientHandler.lcm = lcm;

  if (!lcm) {
    fprintf(stderr, "# ERROR: Cannot initialize LCM.\n");
    exit(EXIT_FAILURE);
  }

  mavconn_mavlink_msg_container_t_subscription_t * comm_sub =
      mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_MAIN,
          &mavlink_handler, lcm);

  GThread* lcm_thread;
  GError* err;

  if (!g_thread_supported()) {
    g_thread_init(NULL);
  }

  if ((lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE,
                                    &err)) == NULL) {
    printf("Thread create failed: %s!!\n", err->message);
    g_error_free(err);
  }

  PxSHMImageClient client;
  client.init(true, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT);
  //client.init(true, PxSHM::CAMERA_FORWARD_LEFT);

  /*ok = true;
  if (ok)
    gui = agui = ok = verbose = true;
*/

  if (verbose)
    fprintf(stderr, "# INFO: Image client ready, waiting for images...\n");

  ///////////////////////////////////////////////////////////////////////////
  KF.transitionMatrix = *(Mat_<float>(2, 2) << 1, 0, 0, 1);
  measurement.setTo(Scalar(0));

  KF.statePre.at<float>(0) = 0;
  KF.statePre.at<float>(1) = 0;

  setIdentity(KF.measurementMatrix);
  setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
  setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
  setIdentity(KF.errorCovPost, Scalar::all(.1));
  ///////////////////////////////////////////////////////////////////////////

  clientHandler.imageClient = &client;

  mavconn_mavlink_msg_container_t_subscription_t* imgSub =
      mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_IMAGES,
          &imageHandler, &clientHandler);

  signal(SIGINT, signalHandler);

  while (!quit) {
    lcm_handle(lcm);
  }

  mavconn_mavlink_msg_container_t_unsubscribe(lcm, imgSub);
  mavconn_mavlink_msg_container_t_unsubscribe(lcm, comm_sub);
  lcm_destroy(lcm);
  g_thread_join(lcm_thread);

  return 0;
}

