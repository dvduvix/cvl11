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

	printf("Go to: X: %f, Y: %f, Z: %f \n", p[0], p[1], p[2]);

	mavlink_msg_set_local_position_setpoint_encode(getSystemID(), compid, &msg,
	                                               &pos);
	sendMAVLinkMessage(lcm, &msg);

	return 0;
}

Vec3f Control::determinePosByDistance(const mavlink_message_t *msg,
                                      PxSHMImageClient *client, Vec3f p) {
  float keep = 1.1f;

  float x, y, z;

  client->getGroundTruth(msg, x, y, z);

  Vec3f g(x * 1000, y * 1000, z * 1000);

  printf("Face center: X: %f, Y: %f, Z: %f \n", p[0], p[1], p[2]);

  Vec3f v = p - g;

  float D = sqrt(v[0] * v[0] + v[1] * v[1]);

  printf("Distance: %f \n", D);

  v = Vec3f(v[0] / D, v[1] / D, 0);

  if (D > keep) {
    v = v * (D - keep);
  } else {
    v = - v * (keep - D);
  }

  Vec3f P = g + v;

  return P;
}

int Control::keepDistance(const mavlink_message_t *msg,
                          PxSHMImageClient *client, Vec3f p, float yaw,
                          lcm_t *lcm, int compid) {
  flyToPos(determinePosByDistance(msg, client, p), yaw, lcm, compid);

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
