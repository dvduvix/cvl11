/*
 * Control.h
 *
 *  Created on: Dec 14, 2011
 *      Author: Vasilevki Dushan
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define FLAG 9999

#include <opencv2/imgproc/imgproc.hpp>
#include "mavconn.h"
#include <interface/shared_mem/PxSHMImageClient.h>

using namespace std;
using namespace cv;

class Control {
 public:
	Control();
	virtual ~Control();

	int flyToPos(Vec3f p, float yaw, lcm_t *lcm, int compid);
	Vec3f determinePosByDistance(const mavlink_message_t *msg,
	                           PxSHMImageClient *client, Vec3f p);
	int keepDistance(const mavlink_message_t *msg, PxSHMImageClient *client,
	                 Vec3f p, float yaw, lcm_t *lcm, int compid);
	Vec3f loopAround(const mavlink_message_t *msg, PxSHMImageClient *client,
	                 Vec3f ap, float rate, lcm_t *lcm, int compid);
	int trackFace(const mavlink_message_t *msg, PxSHMImageClient *client,
                Vec3f objectPosition, Vec3f normal, float fixed_z, lcm_t *lcm,
                int compid);

 private:
	bool validatePosition(Vec3f destination);
  float arcTan(float x, float y);

	Vec3f lastPosition;
};

#endif /* CONTROL_H_ */
