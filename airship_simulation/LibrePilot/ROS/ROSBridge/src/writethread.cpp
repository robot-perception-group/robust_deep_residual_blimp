/*
 ******************************************************************************
 * @addtogroup UAVOROSBridge UAVO to ROS Bridge Module
 * @{
 *
 * @file       writethread.cpp
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             Max Planck Institute for intelligent systems, http://www.is.mpg.de Copyright (C) 2016.
 * @brief      Bridges certain UAVObjects to ROS on USB VCP
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include "rosbridge.h"
#include "std_msgs/String.h"
#include "librepilot/LibrepilotActuators.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "uav_msgs/uav_pose.h"
#include <sstream>
#include "boost/thread.hpp"
#include "writethread.h"
#include "uavorosbridgemessage_priv.h"
#include "pios.h"


namespace librepilot {
class writethread_priv {
public:
    boost::thread *thread;
    ros::NodeHandle *nodehandle;
    rosbridge *parent;

    void offsetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
    {
        offset3d offset;

        offset.x = msg->pose.pose.position.x;
        offset.y = msg->pose.pose.position.y;
        offset.z = msg->pose.pose.position.z;
        parent->setOffset(offset);
    }

    void poseCallback(const geometry_msgs::Vector3Stamped::ConstPtr & msg)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_pos_estimate_t *payload = (rosbridgemessage_pos_estimate_t *)message->data;

        payload->position[0] = msg->vector.x;
        payload->position[1] = msg->vector.y;
        payload->position[2] = msg->vector.z;
        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_POS_ESTIMATE;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        parent->rosinfoPrint("received position, sending");
    }

    void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr & msg)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_vel_estimate_t *payload = (rosbridgemessage_vel_estimate_t *)message->data;

        payload->velocity[0] = msg->vector.x;
        payload->velocity[1] = msg->vector.y;
        payload->velocity[2] = msg->vector.z;
        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_VEL_ESTIMATE;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        parent->rosinfoPrint("received velocity, sending");
    }

    void actuatorCallback(const librepilot::LibrepilotActuators::ConstPtr & msg)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_actuators_t *payload = (rosbridgemessage_actuators_t *)message->data;

        for (int t = 0; t < msg->data.data.size() and t < 12; t++) {
            payload->pwm[t] = (uint16_t)(msg->data.data[t]);
        }
        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_ACTUATORS;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        parent->rosinfoPrint("received actuators, sending");
    }

    void fullstateCallback(const uav_msgs::uav_pose::ConstPtr & msg)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_fullstate_estimate_t *payload = (rosbridgemessage_fullstate_estimate_t *)message->data;

        payload->position[0]   = msg->position.x;
        payload->position[1]   = msg->position.y;
        payload->position[2]   = msg->position.z;
        payload->velocity[0]   = msg->velocity.x;
        payload->velocity[1]   = msg->velocity.y;
        payload->velocity[2]   = msg->velocity.z;
        payload->rotation[0]   = msg->angVelocity.x;
        payload->rotation[1]   = msg->angVelocity.y;
        payload->rotation[2]   = msg->angVelocity.z;
        payload->quaternion[0] = msg->orientation.w;
        payload->quaternion[1] = msg->orientation.x;
        payload->quaternion[2] = msg->orientation.y;
        payload->quaternion[3] = msg->orientation.z;
        payload->accessory[0]  = msg->POI.x; // used for acceleration
        payload->accessory[1]  = msg->POI.y;
        payload->accessory[2]  = msg->POI.z;
        payload->accessory[3]  = msg->thrust; // used for airspeed

        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_FULLSTATE_ESTIMATE;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        parent->rosinfoPrint("hitl state, sending");
    }

    void commandCallback(const uav_msgs::uav_pose::ConstPtr & msg)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_flightcontrol_t *payload = (rosbridgemessage_flightcontrol_t *)message->data;

        offset3d offset = parent->getOffset();
        uint8_t mode    = msg->flightmode;

        if (mode >= ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_NUMELEM) {
            mode = ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_WAYPOINT;
        }
        switch (mode) {
        case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_WAYPOINT:
        case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_VECTOR:
            payload->control[0] = msg->position.x - offset.x;
            payload->control[1] = msg->position.y - offset.y;
            payload->control[2] = msg->position.z - offset.z;
            payload->control[3] = 0;
            payload->vel[0]     = msg->velocity.x;
            payload->vel[1]     = msg->velocity.y;
            payload->vel[2]     = msg->velocity.z;
            break;
        case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ROSCONTROL:
            payload->control[0] = msg->velocity.x;
            payload->control[1] = msg->velocity.y;
            payload->control[2] = msg->velocity.z;
            payload->control[3] = msg->thrust;
            payload->vel[0]     = msg->angVelocity.x;
            payload->vel[1]     = msg->angVelocity.y;
            payload->vel[2]     = msg->angVelocity.z;
            break;
        case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ACTUATORS:
            payload->control[0] = 0;
            payload->control[1] = 0;
            payload->control[2] = 0;
            payload->control[3] = 0;
            payload->vel[0]     = 0;
            payload->vel[1]     = 0;
            payload->vel[2]     = 0;
            break;
        case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ATTITUDE:
            payload->control[0] = msg->velocity.x;
            payload->control[1] = msg->velocity.y;
            payload->control[2] = msg->velocity.z;
            payload->control[3] = msg->thrust;
            break;
        }
        payload->poi[0]    = msg->POI.x - offset.x;
        payload->poi[1]    = msg->POI.y - offset.y;
        payload->poi[2]    = msg->POI.z - offset.z;
        payload->mode      = mode;
        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_FLIGHTCONTROL;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        parent->rosinfoPrint("control");
    }

    void run()
    {
        ros::Rate rate(0.1);

        ros::Subscriber subscriber0 = nodehandle->subscribe(parent->getNameSpace() + "/auxposition", 10, &writethread_priv::poseCallback, this);
        ros::Subscriber subscriber1 = nodehandle->subscribe(parent->getNameSpace() + "/auxvelocity", 10, &writethread_priv::velocityCallback, this);
        ros::Subscriber subscriber2 = nodehandle->subscribe(parent->getNameSpace() + "/command", 10, &writethread_priv::commandCallback, this);
        ros::Subscriber subscriber3 = nodehandle->subscribe(parent->getNameSpace() + "/offset", 10, &writethread_priv::offsetCallback, this);
        ros::Subscriber subscriber4 = nodehandle->subscribe(parent->getNameSpace() + "/actuatorcommand", 10, &writethread_priv::actuatorCallback, this);
        ros::Subscriber subscriber5 = nodehandle->subscribe(parent->getNameSpace() + "/fakehitlstate", 10, &writethread_priv::fullstateCallback, this);

        offset3d offset;

        offset.x = 0.0;
        offset.y = 0.0;
        offset.z = 0.0;
        parent->setOffset(offset);

        while (ros::ok()) {
            uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
            rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
            rosbridgemessage_pingpong_t *payload = (rosbridgemessage_pingpong_t *)message->data;
            payload->sequence_number = parent->getMySequenceNumber() + 1;
            parent->setMySequenceNumber(payload->sequence_number);
            message->magic     = ROSBRIDGEMAGIC;
            message->type      = ROSBRIDGEMESSAGE_PING;
            message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
            boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
            message->timestamp = diff.total_microseconds();
            message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
            int res = parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
            std_msgs::String msg;
            std::stringstream bla;
            bla << "sending a ping myself " << (unsigned int)payload->sequence_number;
            bla << " write res is  : " << (int)res;
            msg.data = bla.str();
            parent->rosinfoPrint(msg.data.c_str());
            rate.sleep();
        }
    }
};

writethread::writethread(ros::NodeHandle *nodehandle, rosbridge *parent)
{
    instance = new writethread_priv();
    instance->parent     = parent;
    instance->nodehandle = nodehandle;
    instance->thread     = new boost::thread(boost::bind(&writethread_priv::run, instance));
}

writethread::~writethread()
{
    instance->thread->detach();
    delete instance->thread;
    delete instance;
}
}
