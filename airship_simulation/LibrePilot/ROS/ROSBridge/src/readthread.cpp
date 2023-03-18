/*
 ******************************************************************************
 * @addtogroup UAVOROSBridge UAVO to ROS Bridge Module
 * @{
 *
 * @file       readthread.cpp
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
#include "librepilot/AutopilotInfo.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "uav_msgs/uav_pose.h"
#include "sensor_msgs/Imu.h"
#include "librepilot/TransmitterInfo.h"
#include "librepilot/gyro_bias.h"
#include <sstream>
#include "boost/thread.hpp"
#include "anonymoussocket.h"
#include "readthread.h"
#include "uavorosbridgemessage_priv.h"
#include "pios.h"
#include "tf/transform_datatypes.h"

namespace librepilot {
class readthread_priv {
public:
    boost::shared_ptr<anonymoussocket> port;
    boost::thread *thread;
    ros::NodeHandle *nodehandle;
    uint8_t rx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
    size_t rx_length;
    rosbridge *parent;
    ros::Publisher state_pub, state2_pub, uavpose_corrected_pub, uavpose_pub, state4_pub, imu_pub, gyro_bias_pub, actuators_pub, autopilot_info_pub;
    uint32_t sequence;
    uint32_t imusequence;

/**
 * Process incoming bytes from an ROS query thing.
 * @param[in] b received byte
 * @return true if we should continue processing bytes
 */
    void ros_receive_byte(uint8_t b)
    {
        rx_buffer[rx_length] = b;
        rx_length++;
        rosbridgemessage_t *message = (rosbridgemessage_t *)rx_buffer;

        // very simple parser - but not a state machine, just a few checks
        if (rx_length <= offsetof(rosbridgemessage_t, length)) {
            // check (partial) magic number - partial is important since we need to restart at any time if garbage is received
            uint32_t canary = 0xff;
            for (uint32_t t = 1; t < rx_length; t++) {
                canary = (canary << 8) | 0xff;
            }
            if ((message->magic & canary) != (ROSBRIDGEMAGIC & canary)) {
                // parse error, not beginning of message
                rx_length = 0;
                {
                    std_msgs::String msg;
                    std::stringstream bla;
                    bla << "canary failure";
                    msg.data = bla.str();
                    parent->rosinfoPrint(msg.data.c_str());
                }
                return;
            }
        }
        if (rx_length == offsetof(rosbridgemessage_t, timestamp)) {
            if (message->length > (uint32_t)(ROSBRIDGEMESSAGE_BUFFERSIZE - offsetof(rosbridgemessage_t, data))) {
                // parse error, no messages are that long
                rx_length = 0;
                {
                    std_msgs::String msg;
                    std::stringstream bla;
                    bla << "zero length";
                    msg.data = bla.str();
                    parent->rosinfoPrint(msg.data.c_str());
                }
                return;
            }
        }
        if (rx_length == offsetof(rosbridgemessage_t, crc32)) {
            if (message->type >= ROSBRIDGEMESSAGE_END_ARRAY_SIZE) {
                // parse error
                rx_length = 0;
                {
                    std_msgs::String msg;
                    std::stringstream bla;
                    bla << "invalid type" << message->type;
                    msg.data = bla.str();
                    parent->rosinfoPrint(msg.data.c_str());
                }
                return;
            }
            if (message->length != ROSBRIDGEMESSAGE_SIZES[message->type]) {
                // parse error
                rx_length = 0;
                {
                    std_msgs::String msg;
                    std::stringstream bla;
                    bla << "invalid length";
                    msg.data = bla.str();
                    parent->rosinfoPrint(msg.data.c_str());
                }
                return;
            }
        }
        if (rx_length < offsetof(rosbridgemessage_t, data)) {
            // not a parse failure, just not there yet
            return;
        }
        if (rx_length == offsetof(rosbridgemessage_t, data) + ROSBRIDGEMESSAGE_SIZES[message->type]) {
            // complete message received and stored in pointer "message"
            // empty buffer for next message
            rx_length = 0;

            if (PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length) != message->crc32) {
                std_msgs::String msg;
                std::stringstream bla;
                bla << "CRC mismatch";
                msg.data = bla.str();
                parent->rosinfoPrint(msg.data.c_str());
                // crc mismatch
                return;
            }
            switch (message->type) {
            case ROSBRIDGEMESSAGE_PING:
                pong_handler((rosbridgemessage_pingpong_t *)message->data);
                break;
            case ROSBRIDGEMESSAGE_FULLSTATE_ESTIMATE:
                fullstate_estimate_handler(message);
                break;
            case ROSBRIDGEMESSAGE_IMU_AVERAGE:
                imu_average_handler(message);
                break;
            case ROSBRIDGEMESSAGE_GYRO_BIAS:
                gyro_bias_handler(message);
                break;
            case ROSBRIDGEMESSAGE_ACTUATORS:
                actuators_handler(message);
                break;
            case ROSBRIDGEMESSAGE_AUTOPILOT_INFO:
                autopilot_info_handler(message);
                break;
            default:
            {
                std_msgs::String msg;
                std::stringstream bla;
                bla << "received something";
                msg.data = bla.str();
                parent->rosinfoPrint(msg.data.c_str());
            }
                // do nothing at all and discard the message
            break;
            }
        }
    }

    void imu_average_handler(rosbridgemessage_t *message)
    {
        rosbridgemessage_imu_average_t *data = (rosbridgemessage_imu_average_t *)message->data;
        sensor_msgs::Imu imu;

        // boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1));

        imu.header.seq      = sequence++;
        // imu.header.stamp.sec  = diff.total_seconds();
        // imu.header.stamp.nsec = 1000 * (diff.total_microseconds() % 1000000);
        imu.header.stamp    = ros::Time::now();
        imu.header.frame_id = "world";
        if (data->gyrsamples != 1 || data->accsamples != 1) {
            parent->rosinfoPrint("imu message wrong sample count");
        }
        imu.orientation_covariance[0] = -1; // orientation is not a sensor, but a state estimate, see fullstate_estimate
        imu.angular_velocity.x = data->gyro_average[0] * (M_PI / 180.0);
        imu.angular_velocity.y = data->gyro_average[1] * (M_PI / 180.0);
        imu.angular_velocity.z = data->gyro_average[2] * (M_PI / 180.0);
        if (data->gyrsamples < 1) {
            imu.angular_velocity_covariance[0] = -1; // no samples, set to ignore!
        }
        imu.linear_acceleration.x = data->accel_average[0];
        imu.linear_acceleration.y = data->accel_average[1];
        imu.linear_acceleration.z = data->accel_average[2];
        if (data->accsamples < 1) {
            imu.linear_acceleration_covariance[0] = -1; // no samples, set to ignore!
        }
        imu_pub.publish(imu);
        // parent->rosinfoPrint("imu published");
    }

    void gyro_bias_handler(rosbridgemessage_t *message)
    {
        rosbridgemessage_gyro_bias_t *data = (rosbridgemessage_gyro_bias_t *)message->data;
        librepilot::gyro_bias gyrobias;

        // boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1));

        gyrobias.header.seq      = sequence++;
        // gyrobias.header.stamp.sec  = diff.total_seconds();
        // gyrobias.header.stamp.nsec = 1000 * (diff.total_microseconds() % 1000000);
        gyrobias.header.stamp    = ros::Time::now();
        gyrobias.header.frame_id = "world";
        gyrobias.bias.x = data->gyro_bias[0];
        gyrobias.bias.y = data->gyro_bias[1];
        gyrobias.bias.z = data->gyro_bias[2];
        gyro_bias_pub.publish(gyrobias);
        parent->rosinfoPrint("gyrobias published");
    }

    void actuators_handler(rosbridgemessage_t *message)
    {
        rosbridgemessage_actuators_t *data = (rosbridgemessage_actuators_t *)message->data;

        librepilot::LibrepilotActuators actuators;

        actuators.header.seq      = sequence++;
        actuators.header.stamp    = ros::Time::now();
        actuators.header.frame_id = "world";

        actuators.data.layout.dim.push_back(std_msgs::MultiArrayDimension());
        actuators.data.layout.dim[0].size   = 12;
        actuators.data.layout.dim[0].stride = 1;
        actuators.data.layout.dim[0].label  = "Channel";
        for (int t = 0; t < 12; t++) {
            actuators.data.data.push_back((double)data->pwm[t]);
        }
        actuators_pub.publish(actuators);
    }

    void autopilot_info_handler(rosbridgemessage_t *message)
    {
        rosbridgemessage_autopilot_info_t *data = (rosbridgemessage_autopilot_info_t *)message->data;

        librepilot::AutopilotInfo autopilot;

        autopilot.header.seq          = sequence++;
        autopilot.header.stamp        = ros::Time::now();
        autopilot.header.frame_id     = "world";

        autopilot.status = data->status;
        autopilot.fractional_progress = data->fractional_progress;
        autopilot.error = data->error;
        autopilot.pathDirection.x     = data->pathDirection[0];
        autopilot.pathDirection.y     = data->pathDirection[1];
        autopilot.pathDirection.z     = data->pathDirection[2];
        autopilot.pathCorrection.x    = data->pathCorrection[0];
        autopilot.pathCorrection.y    = data->pathCorrection[1];
        autopilot.pathCorrection.z    = data->pathCorrection[2];
        autopilot.pathTime = data->pathTime;
        autopilot.Mode = data->Mode;
        autopilot.ModeParameters[0]   = data->ModeParameters[0];
        autopilot.ModeParameters[1]   = data->ModeParameters[1];
        autopilot.ModeParameters[2]   = data->ModeParameters[2];
        autopilot.ModeParameters[3]   = data->ModeParameters[3];
        autopilot.Start.x = data->Start[0];
        autopilot.Start.y = data->Start[1];
        autopilot.Start.z = data->Start[2];
        autopilot.End.x   = data->End[0];
        autopilot.End.y   = data->End[1];
        autopilot.End.z   = data->End[2];
        autopilot.StartingVelocity  = data->StartingVelocity;
        autopilot.EndingVelocity    = data->EndingVelocity;
        autopilot.VelocityDesired.x = data->VelocityDesired[0];
        autopilot.VelocityDesired.y = data->VelocityDesired[1];
        autopilot.VelocityDesired.z = data->VelocityDesired[2];

        autopilot_info_pub.publish(autopilot);
    }

    void fullstate_estimate_handler(rosbridgemessage_t *message)
    {
        rosbridgemessage_fullstate_estimate_t *data = (rosbridgemessage_fullstate_estimate_t *)message->data;
        nav_msgs::Odometry odometry;
        geometry_msgs::PoseStamped pose;
        uav_msgs::uav_pose uavpose;
        librepilot::TransmitterInfo transmitter;

        odometry.header.stamp = ros::Time::now();
        // boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1));

        // ATTENTION:  LibrePilot - like most outdoor platforms uses North-East-Down coordinate frame for all data
        // in body frame
        // x points forward
        // y points right
        // z points down
        // this also goes for the quaternion


        uavpose.position.x    = data->position[0];
        uavpose.position.y    = data->position[1];
        uavpose.position.z    = data->position[2];
        uavpose.velocity.x    = data->velocity[0];
        uavpose.velocity.y    = data->velocity[1];
        uavpose.velocity.z    = data->velocity[2];
        uavpose.orientation.x = data->quaternion[1];
        uavpose.orientation.y = data->quaternion[2];
        uavpose.orientation.z = data->quaternion[3];
        uavpose.orientation.w = data->quaternion[0];
        uavpose.angVelocity.x = data->rotation[0];
        uavpose.angVelocity.y = data->rotation[1];
        uavpose.angVelocity.z = data->rotation[2];
        uavpose.thrust = data->thrust;
        uavpose.flightmode    = data->ROSControlled;
        uavpose.POI.x = data->airspeed;
        for (int t = 0; t < 100; t++) {
            uavpose.covariance[t] = data->matrix[t];
        }

        transmitter.ROSControlled    = data->ROSControlled;
        transmitter.Armed = data->armed;
        transmitter.FlightModeSwitch = data->FlightMode;
        transmitter.Roll         = data->controls[0];
        transmitter.Pitch        = data->controls[1];
        transmitter.Yaw          = data->controls[2];
        transmitter.Thrust       = data->controls[3];
        transmitter.Collective   = data->controls[4];
        transmitter.Throttle     = data->controls[5];
        transmitter.Accessory[0] = data->accessory[0];
        transmitter.Accessory[1] = data->accessory[1];
        transmitter.Accessory[2] = data->accessory[2];
        transmitter.Accessory[3] = data->accessory[3];

        // ROS uses the annozing east-north-up coordinate frame which means axis need to be swapped and signs inverted
        odometry.pose.pose.position.y = data->position[0];
        odometry.pose.pose.position.x = data->position[1];
        odometry.pose.pose.position.z = -data->position[2];

        tf::Quaternion q_0(data->quaternion[1], -data->quaternion[2], -data->quaternion[3], data->quaternion[0]);
        tf::Quaternion q_1, q_2;
        q_1.setEuler(0, 0, M_PI / 2.0);
        quaternionTFToMsg((q_1 * q_0).normalize(), odometry.pose.pose.orientation);
        // quaternionTFToMsg(q_0,odometry.pose.pose.orientation);

        odometry.twist.twist.linear.y  = data->velocity[0];
        odometry.twist.twist.linear.x  = data->velocity[1];
        odometry.twist.twist.linear.z  = -data->velocity[2];
        odometry.twist.twist.angular.y = data->rotation[0] * M_PI / 180;
        odometry.twist.twist.angular.x = data->rotation[1] * M_PI / 180;
        odometry.twist.twist.angular.z = -data->rotation[2] * M_PI / 180;
        // FAKE covariance -- LibrePilot does have a covariance matrix, but its 13x13 and not trivially comparable
        // also ROS documentation on how the covariance is encoded into this double[36] (ro wvs col major, order of members, ...)
        // is very lacing
        for (int t = 0; t < 6; t++) {
            for (int t2 = 0; t < 6; t++) {
                if (t == t2) {
                    odometry.twist.covariance[t * 6 + t2] = 1.0;
                } else {
                    odometry.twist.covariance[t * 6 + t2] = 0.0;
                }
            }
        }
        odometry.header.seq      = sequence++;
        odometry.header.frame_id = "world";
        odometry.child_frame_id  = "copter";
        pose.header        = odometry.header;
        uavpose.header     = odometry.header;
        transmitter.header = odometry.header;
        pose.pose = odometry.pose.pose;

        state_pub.publish(odometry);
        state2_pub.publish(pose);
        uavpose_pub.publish(uavpose);
        offset3d offset = parent->getOffset();
        uavpose.position.x += offset.x;
        uavpose.position.y += offset.y;
        uavpose.position.z += offset.z;
        uavpose_corrected_pub.publish(uavpose);
        state4_pub.publish(transmitter);
        // parent->rosinfoPrint("state published");
    }

    void pong_handler(rosbridgemessage_pingpong_t *data)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_pingpong_t *payload = (rosbridgemessage_pingpong_t *)message->data;

        *payload = *data;
        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_PONG;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        int res = parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        std_msgs::String msg;
        std::stringstream bla;
        bla << "received ping : " << (unsigned int)payload->sequence_number;
        bla << " pong write res is  : " << (int)res;
        msg.data = bla.str();
        parent->rosinfoPrint(msg.data.c_str());
        // chatter_pub.publish(msg);
    }


    void run()
    {
        unsigned char c;

        rx_length     = 0;
        state_pub     = nodehandle->advertise<nav_msgs::Odometry>(parent->getNameSpace() + "/Octocopter", 10);
        state2_pub    = nodehandle->advertise<geometry_msgs::PoseStamped>(parent->getNameSpace() + "/octoPose", 10);
        uavpose_pub   = nodehandle->advertise<uav_msgs::uav_pose>(parent->getNameSpace() + "/pose/raw", 10);
        uavpose_corrected_pub = nodehandle->advertise<uav_msgs::uav_pose>(parent->getNameSpace() + "/pose", 10);
        state4_pub    = nodehandle->advertise<librepilot::TransmitterInfo>(parent->getNameSpace() + "/TransmitterInfo", 10);
        imu_pub       = nodehandle->advertise<sensor_msgs::Imu>(parent->getNameSpace() + "/Imu", 10);
        gyro_bias_pub = nodehandle->advertise<librepilot::gyro_bias>(parent->getNameSpace() + "/gyrobias", 10);
        actuators_pub = nodehandle->advertise<librepilot::LibrepilotActuators>(parent->getNameSpace() + "/actuators", 10);
        autopilot_info_pub = nodehandle->advertise<librepilot::AutopilotInfo>(parent->getNameSpace() + "/AutopilotInfo", 10);
        while (ros::ok()) {
            port->read(&c, 1);
            ros_receive_byte(c);
/*
            std_msgs::String msg;
            std::stringstream bla;
            bla << std::hex << (unsigned short)c;
            msg.data = bla.str();
            parent->rosinfoPrint(msg.data.c_str());
 */
            // chatter_pub.publish(msg);
        }
    }
};

readthread::readthread(ros::NodeHandle *nodehandle, boost::shared_ptr<anonymoussocket> port, rosbridge *parent)
{
    instance = new readthread_priv();
    instance->parent      = parent;
    instance->port        = port;
    instance->nodehandle  = nodehandle;
    instance->sequence    = 0;
    instance->imusequence = 0;
    instance->thread      = new boost::thread(boost::bind(&readthread_priv::run, instance));
}

readthread::~readthread()
{
    instance->thread->detach();
    delete instance->thread;
    delete instance;
}
}
