/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define M_2PI 6.28
#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <tf/transform_datatypes.h>

#include <uav_msgs/uav_pose.h>

double waypoint[3], poi[3], firstPosition[3], currentPosition[3]; double roll, pitch, currentYaw;
double desired_yaw;
bool firstTimePositionCallback = true;
double yawRateGain = 0.5;
double MaxYawRate = 0.1;

void wayPointCallback(const uav_msgs::uav_pose::ConstPtr& msg)
{
    waypoint[0] = msg->position.x;
    waypoint[1] = -msg->position.y;
    waypoint[2] = -msg->position.z;

    ROS_INFO("Waypoint : x:%f, y:%f, z:%f", waypoint[0],  waypoint[1],  waypoint[2]);

    poi[0] = msg->POI.x;
    poi[1] = -msg->POI.y;
    poi[2] = -msg->POI.z;
    // poi[0] = 1;
    // poi[1] = 0;
    // poi[2] = 0;
}

void selfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if(firstTimePositionCallback)
    {
        firstPosition[0] = msg->pose.pose.position.x;
        firstPosition[1] = msg->pose.pose.position.y;
        firstPosition[2] = msg->pose.pose.position.z;
        firstTimePositionCallback = false;
    }
    currentPosition[0] = msg->pose.pose.position.x;
    currentPosition[1] = msg->pose.pose.position.y;
    currentPosition[2] = msg->pose.pose.position.z;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, currentYaw);
    //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;


}

	static inline double normPi(double a) {
		if (a > M_PI || a <= -M_PI) {
			a = fmod(a, M_2PI); //in [-2*M_PI,2*M_PI]
			if (a < -M_PI)
                                a += M_2PI;
			if (a > M_PI)
				a -= M_2PI;
		}
		return a;
	}

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started waypoint_publisher.");



  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  int selfID = atoi(argv[1]);

  ros::Subscriber subNMPCwaypoint_ = nh.subscribe("/waypoint_firefly_"+args.at(1), 1000, wayPointCallback);

  ros::Subscriber subSelfPose_ = nh.subscribe("/firefly_"+args.at(1)+"/ground_truth/pose_with_covariance", 1000, selfPoseCallback);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  } else {
    ROS_ERROR("Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]\n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;
  const float takeOffHeight = 2.5; //in meter

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();



  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }

// <<<<<<< HEAD
  ros::Time takeOffStartTime = ros::Time::now();
  ros::Rate loop_rate(100);
  while (ros::ok())
  {

    if((ros::Time::now() - takeOffStartTime).toSec() < 10.0)  //perform takeoff
    {
        Eigen::Vector3d desired_position(firstPosition[0],firstPosition[1],takeOffHeight);
        double desired_yaw = atan2(poi[1]-firstPosition[1],poi[0]-firstPosition[0]);
        desired_yaw = 0.0;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);
        //ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",nh.getNamespace().c_str(),desired_position.x(),desired_position.y(),desired_position.z());
        trajectory_pub.publish(trajectory_msg);
    }
    else
    {

        Eigen::Vector3d desired_position(waypoint[0],waypoint[1],waypoint[2]);

        desired_yaw = atan2(poi[1]-waypoint[1],poi[0]-waypoint[0]);



        double desiredYawRate = yawRateGain*(normPi(desired_yaw - currentYaw));
        if (fabs(desiredYawRate) > MaxYawRate) {
            desiredYawRate = copysign(MaxYawRate, desiredYawRate);
        }

        double diffYaw = (desired_yaw - currentYaw);
        diffYaw = normPi(diffYaw);

        if(diffYaw>=M_PI/2)
            desired_yaw = desired_yaw - M_PI/4;
        if(diffYaw<=-M_PI/2)
            desired_yaw = desired_yaw + M_PI/4;

//         if(selfID == 1)
//         {
//             std::cout<<"desired_yaw - currentYaw for robot "<< selfID  <<"  = " <<desired_yaw - currentYaw<<std::endl;
//             //std::cout<<"desired_yaw for robot "<< selfID  <<"  = " <<desired_yaw<<std::endl;
//         }


        /*mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);*/
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);

        //ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",nh.getNamespace().c_str(),desired_position.x(),desired_position.y(),desired_position.z());

        trajectory_pub.publish(trajectory_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

// // =======
//   ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
//            nh.getNamespace().c_str(),
//            desired_position.x(),
//            desired_position.y(),
//            desired_position.z());
//
//   trajectory_pub.publish(trajectory_msg);
// >>>>>>> c4de36092f489c46c9cb056765d0ef5c7919919a

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
