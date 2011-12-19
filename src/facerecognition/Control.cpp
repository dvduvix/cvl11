/*
 * Control.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: Vasilevski Dushan
 */

#include "Control.h"

Control::Control() { }

Control::~Control() { }

int Control::flyToPos(Vec3f p, float yaw, lcm_t *lcm, int compid) {
	mavlink_message_t msg;
	mavlink_set_local_position_setpoint_t pos;

	pos.x   = p[0];
	pos.y   = p[1];
	pos.z   = p[2];
	pos.yaw = yaw;
	pos.target_system     = getSystemID();
	pos.target_component  = 200;
	pos.coordinate_frame = 1;

	mavlink_msg_set_local_position_setpoint_encode(getSystemID(), compid, &msg,
	                                               &pos);
	sendMAVLinkMessage(lcm, &msg);

	return 0;
}

Vec3f Control::determinePosByDistance(const mavlink_message_t *msg,
                                      PxSHMImageClient *client, Vec3f p) {
  float x, y, z;

  client->getGroundTruth(msg, x, y, z);

  Vec3f g(x, y, z);
  Vec3f v = p - g;

  float D = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

  v = Vec3f(v[0] / D, v[1] / D, v[2] / D);
  v = v * (D - 1);

  Vec3f P = g + v;

  return P;
}

int Control::keepDistance(const mavlink_message_t *msg,
                          PxSHMImageClient *client, Vec3f p, lcm_t *lcm,
                          int compid) {
  flyToPos(determinePosByDistance(msg, client, p), 0, lcm, compid);

  return 0;
}

Vec3f Control::loopAround(const mavlink_message_t *msg, PxSHMImageClient *client,
                        Vec3f ap, float rate, lcm_t *lcm, int compid) {

  float x, y, z;
  float roll, pitch, yaw;

  client->getGroundTruth(msg, x, y, z);
  client->getRollPitchYaw(msg, roll, pitch, yaw);

  float a, b, r;

  a = ap[0];
  b = ap[1];
  r = sqrt((x - a) * (x - a) + (y - b) * (y - b));

  Vec3f p(a + r * cos(rate), b + r * sin(rate), z);

  rate = yaw + rate;

  return p;
}
