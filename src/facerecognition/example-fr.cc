/**
 * @file
 *   @brief Face Tracking
 *
 *   @author Vasilevski Dushan
 *
 */

#define NONO_DISPLAY 1

#include <cstdio>
#include <unistd.h>
#include <glib.h>
#include "mavconn.h"
#include "Face.h"
#include "World.h"
#include "StereoProc.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <signal.h>

#include <interface/shared_mem/PxSHMImageClient.h>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

// Timer for benchmarking
struct timeval tv;

int sysid = 42;
int compid = 112;
bool verbose = false;
bool debug = 0;

static GString* configFile = g_string_new("config.cfg");
string stereoCfg;

int imageCounter = 0;
std::string fileBaseName("frame");
std::string fileExt(".png");

bool quit = false;

using namespace std;
using namespace cv;

// Global variables
Face *face;
World *world;
StereoProc stereo;

bool foundFace = false;
bool isInit = true;

float colormap_jet[128][3] =
{
		{0.0f,0.0f,0.53125f},
		{0.0f,0.0f,0.5625f},
		{0.0f,0.0f,0.59375f},
		{0.0f,0.0f,0.625f},
		{0.0f,0.0f,0.65625f},
		{0.0f,0.0f,0.6875f},
		{0.0f,0.0f,0.71875f},
		{0.0f,0.0f,0.75f},
		{0.0f,0.0f,0.78125f},
		{0.0f,0.0f,0.8125f},
		{0.0f,0.0f,0.84375f},
		{0.0f,0.0f,0.875f},
		{0.0f,0.0f,0.90625f},
		{0.0f,0.0f,0.9375f},
		{0.0f,0.0f,0.96875f},
		{0.0f,0.0f,1.0f},
		{0.0f,0.03125f,1.0f},
		{0.0f,0.0625f,1.0f},
		{0.0f,0.09375f,1.0f},
		{0.0f,0.125f,1.0f},
		{0.0f,0.15625f,1.0f},
		{0.0f,0.1875f,1.0f},
		{0.0f,0.21875f,1.0f},
		{0.0f,0.25f,1.0f},
		{0.0f,0.28125f,1.0f},
		{0.0f,0.3125f,1.0f},
		{0.0f,0.34375f,1.0f},
		{0.0f,0.375f,1.0f},
		{0.0f,0.40625f,1.0f},
		{0.0f,0.4375f,1.0f},
		{0.0f,0.46875f,1.0f},
		{0.0f,0.5f,1.0f},
		{0.0f,0.53125f,1.0f},
		{0.0f,0.5625f,1.0f},
		{0.0f,0.59375f,1.0f},
		{0.0f,0.625f,1.0f},
		{0.0f,0.65625f,1.0f},
		{0.0f,0.6875f,1.0f},
		{0.0f,0.71875f,1.0f},
		{0.0f,0.75f,1.0f},
		{0.0f,0.78125f,1.0f},
		{0.0f,0.8125f,1.0f},
		{0.0f,0.84375f,1.0f},
		{0.0f,0.875f,1.0f},
		{0.0f,0.90625f,1.0f},
		{0.0f,0.9375f,1.0f},
		{0.0f,0.96875f,1.0f},
		{0.0f,1.0f,1.0f},
		{0.03125f,1.0f,0.96875f},
		{0.0625f,1.0f,0.9375f},
		{0.09375f,1.0f,0.90625f},
		{0.125f,1.0f,0.875f},
		{0.15625f,1.0f,0.84375f},
		{0.1875f,1.0f,0.8125f},
		{0.21875f,1.0f,0.78125f},
		{0.25f,1.0f,0.75f},
		{0.28125f,1.0f,0.71875f},
		{0.3125f,1.0f,0.6875f},
		{0.34375f,1.0f,0.65625f},
		{0.375f,1.0f,0.625f},
		{0.40625f,1.0f,0.59375f},
		{0.4375f,1.0f,0.5625f},
		{0.46875f,1.0f,0.53125f},
		{0.5f,1.0f,0.5f},
		{0.53125f,1.0f,0.46875f},
		{0.5625f,1.0f,0.4375f},
		{0.59375f,1.0f,0.40625f},
		{0.625f,1.0f,0.375f},
		{0.65625f,1.0f,0.34375f},
		{0.6875f,1.0f,0.3125f},
		{0.71875f,1.0f,0.28125f},
		{0.75f,1.0f,0.25f},
		{0.78125f,1.0f,0.21875f},
		{0.8125f,1.0f,0.1875f},
		{0.84375f,1.0f,0.15625f},
		{0.875f,1.0f,0.125f},
		{0.90625f,1.0f,0.09375f},
		{0.9375f,1.0f,0.0625f},
		{0.96875f,1.0f,0.03125f},
		{1.0f,1.0f,0.0f},
		{1.0f,0.96875f,0.0f},
		{1.0f,0.9375f,0.0f},
		{1.0f,0.90625f,0.0f},
		{1.0f,0.875f,0.0f},
		{1.0f,0.84375f,0.0f},
		{1.0f,0.8125f,0.0f},
		{1.0f,0.78125f,0.0f},
		{1.0f,0.75f,0.0f},
		{1.0f,0.71875f,0.0f},
		{1.0f,0.6875f,0.0f},
		{1.0f,0.65625f,0.0f},
		{1.0f,0.625f,0.0f},
		{1.0f,0.59375f,0.0f},
		{1.0f,0.5625f,0.0f},
		{1.0f,0.53125f,0.0f},
		{1.0f,0.5f,0.0f},
		{1.0f,0.46875f,0.0f},
		{1.0f,0.4375f,0.0f},
		{1.0f,0.40625f,0.0f},
		{1.0f,0.375f,0.0f},
		{1.0f,0.34375f,0.0f},
		{1.0f,0.3125f,0.0f},
		{1.0f,0.28125f,0.0f},
		{1.0f,0.25f,0.0f},
		{1.0f,0.21875f,0.0f},
		{1.0f,0.1875f,0.0f},
		{1.0f,0.15625f,0.0f},
		{1.0f,0.125f,0.0f},
		{1.0f,0.09375f,0.0f},
		{1.0f,0.0625f,0.0f},
		{1.0f,0.03125f,0.0f},
		{1.0f,0.0f,0.0f},
		{0.96875f,0.0f,0.0f},
		{0.9375f,0.0f,0.0f},
		{0.90625f,0.0f,0.0f},
		{0.875f,0.0f,0.0f},
		{0.84375f,0.0f,0.0f},
		{0.8125f,0.0f,0.0f},
		{0.78125f,0.0f,0.0f},
		{0.75f,0.0f,0.0f},
		{0.71875f,0.0f,0.0f},
		{0.6875f,0.0f,0.0f},
		{0.65625f,0.0f,0.0f},
		{0.625f,0.0f,0.0f},
		{0.59375f,0.0f,0.0f},
		{0.5625f,0.0f,0.0f},
		{0.53125f,0.0f,0.0f},
		{0.5f,0.0f,0.0f}
};

void colorDepthImage(cv::Mat& imgDepth, cv::Mat& imgColoredDepth)
{
	imgColoredDepth = cv::Mat::zeros(imgDepth.size(), CV_8UC3);

	for (int i = 0; i < imgColoredDepth.rows; ++i)
	{
		const float* depth = imgDepth.ptr<float>(i);
		unsigned char* pixel = imgColoredDepth.ptr<unsigned char>(i);
		for (int j = 0; j < imgColoredDepth.cols; ++j)
		{
			if (depth[j] != 0)
			{
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

void
signalHandler(int signal)
{
	if (signal == SIGINT)
	{
		fprintf(stderr, "# INFO: Quitting...\n");
		quit = true;
		exit(EXIT_SUCCESS);
	}
}

/**
 * @brief Handle incoming MAVLink packets containing images
 *
 */
void imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
		const mavconn_mavlink_msg_container_t* container, void* user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Pointer to shared memory data
	PxSHMImageClient* client = static_cast<PxSHMImageClient*>(user);

	cv::Mat imgToSave;

	cv::Mat imgL, imgR, imgDepth, imgRectified;

	bool s = stereo.init(stereoCfg);

	if (!s) {
		printf("ERROR: Stereo not init-ed\n");
		exit(EXIT_FAILURE);
	}

	cv::Mat intrinsicMat;
	cv::Mat imgDepthColor;

	stereo.getImageInfo(intrinsicMat);
	float focus = intrinsicMat.at<float>(0, 0);

	if (client->readStereoImage(msg, imgL, imgR)) {

		if (isInit) {
			face->init(imgL);
			isInit = false;
		}

		foundFace =	face->detectFace(imgL);

		if (foundFace) {
			stereo.process(imgL, imgR, imgRectified, imgDepth);
			cv::Mat *imgDepthCopy = new cv::Mat();
			imgDepth.copyTo(*imgDepthCopy);

			colorDepthImage(imgDepth, imgDepthColor);
			cv::namedWindow("Depth map");
			cv::imshow("Depth map", imgDepthColor);

			float roll, pitch, yaw;
			client->getRollPitchYaw(msg, roll, pitch, yaw);
			world->constructRotationMatrix(roll, pitch, yaw);

//			printf("%d %d \n", face->faceProp.p1.x, face->faceProp.p1.y);
//			printf("%d %d \n", face->faceProp.p2.x, face->faceProp.p2.y);
//			printf("%d %d \n", face->faceProp.p3.x, face->faceProp.p3.y);
			world->normalFromPoints(world->get3DPoint(face->faceProp.p1, imgDepth, focus),
									world->get3DPoint(face->faceProp.p2, imgDepth, focus),
									world->get3DPoint(face->faceProp.p3, imgDepth, focus));


			IplImage iImgL = imgL;

			cvLine(&iImgL, face->faceProp.p1, face->faceProp.p2,
					cvScalar(255, 0, 0, 1), 2);
			cvLine(&iImgL, face->faceProp.p1, face->faceProp.p3,
					cvScalar(255, 0, 0, 1));
			cvLine(&iImgL, face->faceProp.p2, face->faceProp.p3,
					cvScalar(255, 0, 0, 1), 2);

		} else {
			switch (face->direction) {
				case Face::D :
					printf("V V Down ... \n");
				break;
				case Face::U :
					printf("^ ^ Up ... \n");
				break;
				case Face::L :
					printf("< < Left ... \n");
				break;
				case Face::R :
					printf("> > Right ... \n");
				break;
				case Face::UL :
					printf("^ < Up Left ... \n");
				break;
				case Face::UR :
					printf("^ > Up Right ... \n");
				break;
				case Face::DL :
					printf("V < Down Left... \n");
				break;
				case Face::DR :
					printf("V > Down Right... \n");
				break;
			}
		}


		/*struct timeval tv;
		gettimeofday(&tv, NULL);
		uint64_t currTime = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
		uint64_t timestamp = client->getTimestamp(msg);

		uint64_t diff = currTime - timestamp;

		cout << diff;

		if (verbose)
		{
			fprintf(stderr, "# INFO: Time from capture to display: %llu ms for camera %llu\n", diff / 1000, client->getCameraID(msg));
		}*/

		// Display if switched on
#ifndef NO_DISPLAY
		if ((client->getCameraConfig() & PxSHM::CAMERA_FORWARD_LEFT) == PxSHM::CAMERA_FORWARD_LEFT)
		{
			cv::namedWindow("Left Image (Forward Camera)");
			cv::imshow("Left Image (Forward Camera)", imgL);
		}
		else
		{
			cv::namedWindow("Left Image (Downward Camera)");
			cv::imshow("Left Image (Downward Camera)", imgL);
		}
#endif

		imgL.copyTo(imgToSave);
	}

#ifndef NO_DISPLAY
	int c = cv::waitKey(3);
	switch (static_cast<char>(c))
	{
	case 'f':
	{
		char index[20];
		sprintf(index, "%04d", imageCounter++);
		cv::imwrite(std::string(fileBaseName+index+fileExt).c_str(), imgToSave);
	}
	break;
	default:
		break;
	}
#endif
}

static void
mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel,
		const mavconn_mavlink_msg_container_t* container, void * user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
	mavlink_message_t response;
	lcm_t* lcm = static_cast<lcm_t*>(user);
	//printf("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n", msg->msgid, channel, msg->sysid, msg->compid);

	switch(msg->msgid)
	{

	uint32_t receiveTime;
	uint32_t sendTime;
/*	case MAVLINK_MSG_ID_COMMAND_SHORT:
	{
		mavlink_command_short_t cmd;
		mavlink_msg_command_short_decode(msg, &cmd);
		/*printf("Message ID: %d\n", msg->msgid);
		printf("Command ID: %d\n", cmd.command);
		printf("Target System ID: %d\n", cmd.target_system);
		printf("Target Component ID: %d\n", cmd.target_component);
		printf("\n");*/

		/*if (cmd.confirmation)
		{
			//printf("Confirmation requested, sending confirmation:\n");
			mavlink_command_ack_t ack;
			ack.command = cmd.command;
			ack.result = 3;
			mavlink_msg_command_ack_encode(getSystemID(), compid, &response, &ack);
			sendMAVLinkMessage(lcm, &response);
		}
	}
	break;*/
	case MAVLINK_MSG_ID_ATTITUDE:
		gettimeofday(&tv, NULL);
		receiveTime = tv.tv_usec;
		sendTime = mavlink_msg_attitude_get_time_boot_ms(msg);
		//printf("Received attitude message, transport took %f ms\n", (receiveTime - sendTime)/1000.0f);
		break;
	case MAVLINK_MSG_ID_GPS_RAW_INT:
	{
		mavlink_gps_raw_int_t gps;
		mavlink_msg_gps_raw_int_decode(msg, &gps);
		//printf("GPS: lat: %f, lon: %f, alt: %f\n", gps.lat/(double)1E7, gps.lon/(double)1E7, gps.alt/(double)1E6);
		break;
	}
	case MAVLINK_MSG_ID_RAW_PRESSURE:
	{
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


void* lcm_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1)
	{
		lcm_handle (lcm);
	}
	return NULL;
}

// Handling Program options
static GOptionEntry entries[] =
{
		{ "sysid", 'a', 0, G_OPTION_ARG_INT, &sysid, "ID of this system, 1-255", "42"},
		{ "compid", 'c', 0, G_OPTION_ARG_INT, &compid, "ID of this component, 1-255", "55" },
		{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", (verbose) ? "true" : "false" },
		{ "debug", 'd', 0, G_OPTION_ARG_NONE, &debug, "Debug mode, changes behaviour", (debug) ? "true" : "false" },
		{ "config", 'g', 0, G_OPTION_ARG_STRING, configFile, "Filename of paramClient config file", "config/parameters_2pt.cfg"},
		{ NULL }
};

int main(int argc, char* argv[])
{

	//////////////////////////////////////////////////////////////////

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

	stereo.init("");
	//////////////////////////////////////////////////////////////////

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- localize based on natural features");
	g_option_context_add_main_entries (context, entries, "Localization");
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit(EXIT_FAILURE);
	}
	g_option_context_free(context);
	// Handling Program options
	lcm_t* lcm = lcm_create("udpm://");
	if (!lcm)
	{
		fprintf(stderr, "# ERROR: Cannot initialize LCM.\n");
		exit(EXIT_FAILURE);
	}

	mavconn_mavlink_msg_container_t_subscription_t * comm_sub =
			mavconn_mavlink_msg_container_t_subscribe (lcm, MAVLINK_MAIN, &mavlink_handler, lcm);

	// Thread
	GThread* lcm_thread;
	GError* err;

	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
	{
		printf("Thread create failed: %s!!\n", err->message );
		g_error_free ( err ) ;
	}

	PxSHMImageClient client;
	client.init(true, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT);
	//client.init(true, PxSHM::CAMERA_FORWARD_LEFT);

	// Ready to roll
	fprintf(stderr, "# INFO: Image client ready, waiting for images...\n");

	// Subscribe to MAVLink messages on the image channel
	mavconn_mavlink_msg_container_t_subscription_t* imgSub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_IMAGES, &imageHandler, &client);

	signal(SIGINT, signalHandler);

	while (!quit)
	{
		// Block waiting for new image messages,
		// once an image is received it will be
		// displayed
		lcm_handle(lcm);
	}

	mavconn_mavlink_msg_container_t_unsubscribe(lcm, imgSub);
	mavconn_mavlink_msg_container_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);
	g_thread_join(lcm_thread);
	return 0;
}

