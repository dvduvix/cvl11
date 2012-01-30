/*
 * Control.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: Vasilevski Dushan
 */

#include "Control.h"

Control::Control() {
  lastPosition[0] = FLAG;
  lastPosition[1] = FLAG;
  lastPosition[2] = FLAG;
}

Control::~Control() { }

/**
  * Sends a MAVLink Message with position and a yaw.
  *
  * @param p Position
  * @param yaw Yaw
  * @param lcm lcm_t *
  * @param compid int
  *
  * @return 0
  */
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
  std::cout << "Yaw: " << pos.yaw << std::endl;

	mavlink_msg_set_local_position_setpoint_encode(getSystemID(), compid, &msg,
	                                               &pos);
	sendMAVLinkMessage(lcm, &msg);

	return 0;
}

/**
  * Determines the position of the setpoint to fly the quad to a constant
  * distance from the face
  *
  * @param msg const mavlink_message_t *
  * @param yaw Yaw
  * @param client PxSHMImageClient *
  * @param p The position of the face in meters
  *
  * @return P The new setpoint
  */
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

/**
  * Keeps the helicopter at a constant distance from the face
  *
  * @param msg const mavlink_message_t *
  * @param client PxSHMImageClient *
  * @param p The position of the face in meters
  * @param yaw The yaw to send
  * @param lcm lam_t *
  * @param compid int
  *
  * @return 0
  */
int Control::keepDistance(const mavlink_message_t *msg,
                          PxSHMImageClient *client, Vec3f p, float yaw,
                          lcm_t *lcm, int compid) {
  flyToPos(determinePosByDistance(msg, client, p), yaw, lcm, compid);

  return 0;
}

/**
  * Loops the quad around one fixed point
  *
  * @param msg const mavlink_message_t *
  * @param client PxSHMImageClient *
  * @param ap The position of the face in meters
  * @param rate At which rate should the quad loop
  * @param lcm lam_t *
  * @param compid int
  *
  * @return p The new setpoint
  */
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

/**
  * Tracks the face. Keeps a fixed distance from the face and makes the quad to
  * face the face. Before sending the message it validates the postion, the
  * normal and the yaw
  *
  * @param msg const mavlink_message_t *
  * @param client PxSHMImageClient *
  * @param objectPosition The position of the face in meters
  * @param normal Normal of the face in meters
  * @param quad_point Location of the quad in meters
  * @param fixed_z A fixed z-coordinate of the set point
  * @param lcm lam_t *
  * @param compid int
  *
  * @return 0
  */
int Control::trackFace(const mavlink_message_t *msg, PxSHMImageClient *client,
                       Vec3f objectPosition, Vec3f normal, Vec3f quad_point,
                       float fixed_z, lcm_t *lcm, int compid) {
  float keep = 1.1f;

  float normalization = sqrt(normal[0] * normal[0] + normal[1] * normal[1]);

  float yaw = arcTan(normal[0], normal[1]);

  Vec3f destination;

  destination[0] = objectPosition[0] - keep * normal[0] / normalization;
  destination[1] = objectPosition[1] - keep * normal[1] / normalization;
  destination[2] = fixed_z;

 // std::cout << "Normal: " << Mat(normal) << std::endl;

  if (validatePosition(destination, yaw, quad_point, objectPosition) &&
      validateNormal(normal))
    flyToPos(destination, yaw, lcm, compid);

  return 0;
}

/**
  * Validates the new set point by checking if the change from the previous
  * is reasonable. Does the same of the yaw.
  *
  * @param destination The set point
  * @param yaw The yaw
  * @param quad_point Location of the quad in meters
  * @param object_point Location of the face
  *
  * @return true/false
  */
bool Control::validatePosition(Vec3f destination, float yaw, Vec3f quad_point,
                               Vec3f object_point) {
  float difference = 0.5f;
  float anglediffr = M_PI_2;

  float relativeAngle = arcTan(object_point[0] - quad_point[0],
                               object_point[1] - quad_point[1]);

  if(lastPosition[0] != FLAG && lastPosition[1] != FLAG) {
    if (sqrt(pow(lastPosition[0] - destination[0], 2.0f) +
             pow(lastPosition[1] - destination[1], 2.0f)) < difference &&
        abs(relativeAngle - yaw) < anglediffr)
      return true;
  } else {
    lastPosition = destination;
  }

  return false;
}

/**
  * Validates if the normal is an actual normal.
  *
  * @param normal The normal
  *
  * @return true/false
  */
bool Control::validateNormal(cv::Vec3f normal) {
  float threshold = 0.2;

  if ((abs(normal[0]) + abs(normal[1])) < threshold)
    return false;

  return true;
}

/**
  * Calculates the yaw for the quad.
  *
  * @param x X-coordinte of the normal
  * @param y Y-coordinte of the normal
  *
  * @return angle The yaw in radians
  */
float Control::arcTan(float x, float y) {
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

  return angle;
}
