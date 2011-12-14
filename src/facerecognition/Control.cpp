/*
 * Control.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: Vasilevski Dushan
 */

#include "Control.h"

Control::Control() { }

Control::~Control() { }

int Control::flyToPos(Point3d p, lcm_t *lcm, int compid)
{
	mavlink_message_t msg;
	mavlink_set_local_position_setpoint_t pos;

	pos.x = p.x;
	pos.y = p.y;
	pos.z = p.z;

	mavlink_msg_set_local_position_setpoint_encode(getSystemID(), compid, &msg, &pos);
	sendMAVLinkMessage(lcm, &msg);
}

