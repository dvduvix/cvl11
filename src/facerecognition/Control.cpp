/*
 * Control.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: Vasilevski Dushan
 */

#include "Control.h"

Control::Control() { }

Control::~Control() { }

int Control::flyToPos(Vec3f p, lcm_t *lcm, int compid)
{
	mavlink_message_t msg;
	mavlink_set_local_position_setpoint_t pos;

	pos.x = p[0];
	pos.y = p[1];
	pos.z = p[2];

	mavlink_msg_set_local_position_setpoint_encode(getSystemID(), compid, &msg, &pos);
	sendMAVLinkMessage(lcm, &msg);

	return 0;
}

