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
	pos.yaw = yaw * 180 / M_PI;
	pos.target_system     = getSystemID();
	pos.target_component  = 200;
	pos.coordinate_frame = 1;

	printf("Go to: X: %f, Y: %f, Z: %f \n", pos.x, pos.y, pos.z);

	mavlink_msg_set_local_position_setpoint_encode(getSystemID(), compid, &msg,
	                                               &pos);
	sendMAVLinkMessage(lcm, &msg);

	return 0;
}

Vec3f Control::determinePosByDistance(const mavlink_message_t *msg,
                                      PxSHMImageClient *client, Vec3f p) {
  float keep = 1500.0f;

  float x, y, z;

  client->getGroundTruth(msg, x, y, z);

  Vec3f g(x * 1000, y * 1000, z * 1000);
  Vec3f v;

  //printf("Face center: X: %f, Y: %f, Z: %f \n", p[0] /1000.0f, p[1] /1000.0f, p[2] / 1000.0f);

  v[0] = p[0] - g[0];
  v[1] = p[1] - g[1];
  v[2] = p[2] - g[2];

  float D = sqrt(v[0] * v[0] + v[1] * v[1]);

  printf("Distance: %f \n", D);

  v = Vec3f(v[0] / D, v[1] / D, 0);

  if (D > keep) {
    v[0] = v[0] * (D - keep);
    v[1] = v[1] * (D - keep);
    v[2] = v[2] * (D - keep);
  } else {
    v[0] = v[0] * (D - keep);
    v[1] = v[1] * (D - keep);
    v[2] = v[2] * (D - keep);
  }

  Vec3f P = g + v;
  P[2] = p[2];
  v[0] = p[0] - P[0];
  v[1] = p[1] - P[1];
  v[2] = p[2] - P[2];

  D = sqrt(v[0] * v[0] + v[1] * v[1]);

   printf("Distance N: %f \n", D);

  P[0] =P[0] / 1000.0f;
  P[1] =P[1] / 1000.0f;

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

int Control::trackFace(const mavlink_message_t *msg, PxSHMImageClient *client,
                       Vec3f objectPosition, Vec3f normal, float fixed_z,
                       lcm_t *lcm, int compid) {
  float keep = 1.5f;

  Point3f quadPosition;

  client->getGroundTruth(msg, quadPosition.x, quadPosition.y, quadPosition.z);

  Vec3f d = objectPosition - Vec3f(quadPosition);

  Point3f distance = Point3f(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]),
                             0, 0);

  float yaw = atan2(objectPosition[1] - quadPosition.y,
                    objectPosition[0] - quadPosition.x);

  float normalization = sqrt(normal[0] * normal[0] + normal[1] * normal[1]);

  Vec3f destination;

  destination[0] = objectPosition[0] - keep * normal[0] / normalization;
  destination[1] = objectPosition[1] - keep * normal[1] / normalization;
  destination[2] = fixed_z;

  float D = sqrt((destination[0] - objectPosition[0]) *
                 (destination[0] - objectPosition[0]) +
                 (destination[1] - objectPosition[1]) *
                 (destination[1] - objectPosition[1]));

  printf("New Distance: %f \n", D);

  flyToPos(destination, yaw, lcm, compid);

  return 0;
}
