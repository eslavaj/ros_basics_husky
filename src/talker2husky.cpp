/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
//#include "turtlesim/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <math.h>
#include <vector>

// %EndTag(MSG_HEADER)%

#include <sstream>


typedef struct HuskyIntPosition
{
  double x;
  double y;
  double z;
}HuskyIntPosition;


HuskyIntPosition huskyIntPosition;

/**
 * This tutorial demonstrates simple sending of and receiving messages over the ROS system.
 */

#define Kw 0.7
#define thetaErrorTH (M_PI*0.5)
#define distanceErrorTH (0.05)

#define Kv 0.45

geometry_msgs::Point currentDestPoint;
std::vector<geometry_msgs::PoseStamped> currentPath;
geometry_msgs::Twist control;
// %Tag(PUBLISHER)%
ros::Publisher vel_pub;
// %EndTag(PUBLISHER)%


double calcPhi(double x, double y, double xp, double yp)
{
  double phi = atan2( (yp-y), (xp-x) );
  return phi;
}


void correctTheta(double phi, double theta)
{
  double errorTheta = theta - phi;
  if(errorTheta > M_PI)
  {
    errorTheta = errorTheta - 2*M_PI;
  }
  else if(errorTheta < -M_PI)
  {
    errorTheta = errorTheta + 2*M_PI;
  }

  double w = -Kw*errorTheta;
  
  control.angular.z = w;

// %Tag(PUBLISH)%
  vel_pub.publish(control);
// %EndTag(PUBLISH)%

}


double calcDistance(double x, double y, double xp, double yp)
{
  double distance = sqrt( powf(x - xp, 2) + powf(y - yp, 2) );
  return distance;
}

void correctDistance(double distance)
{
  control.linear.x = Kv * distance;
// %Tag(PUBLISH)%
  vel_pub.publish(control);
// %EndTag(PUBLISH)%
}


// %Tag(CALLBACK)%
void guideCallback(const nav_msgs::Odometry::ConstPtr& huskyOdometry)
{

  huskyIntPosition.x = huskyOdometry->pose.pose.position.x;
  huskyIntPosition.y = huskyOdometry->pose.pose.position.y;
  huskyIntPosition.z = huskyOdometry->pose.pose.position.z;

  double xp = currentDestPoint.x;
  double yp = currentDestPoint.y;  

  ROS_WARN("The husky position is: [%f %f %f]",huskyIntPosition.x, huskyIntPosition.y, huskyIntPosition.z);

  if(calcDistance(huskyIntPosition.x, huskyIntPosition.y, xp, yp) > distanceErrorTH)
  {

    double qx = huskyOdometry->pose.pose.orientation.x;
    double qy = huskyOdometry->pose.pose.orientation.y;
    double qz = huskyOdometry->pose.pose.orientation.z;
    double qw = huskyOdometry->pose.pose.orientation.w;

    //ROS_WARN("The husky orientation is: [%f %f %f %f]", qx, qy, qz, qw);

    double newTheta = atan2(2*qw*qz, 1 - 2*qz*qz);

    double newPhi = calcPhi(huskyIntPosition.x, huskyIntPosition.y, xp, yp);
    //ROS_WARN("newPhi: [%f]", newPhi);
    correctTheta(newPhi, newTheta);

    if(fabs(newPhi - newTheta)<thetaErrorTH)
    {
      double newDistance = calcDistance(huskyIntPosition.x, huskyIntPosition.y, xp, yp);
      //ROS_WARN("newDistance: [%f]", newDistance);
      correctDistance(newDistance);
    }

  }

}
// %EndTag(CALLBACK)%


// %Tag(CALLBACK)%
void updtTargetPointCallback(const geometry_msgs::Point::ConstPtr& newPoint)
{
  ROS_WARN("New target point is: [%f %f]", newPoint->x, newPoint->y);
  currentDestPoint.x = newPoint->x;
  currentDestPoint.y = newPoint->y;
}
// %EndTag(CALLBACK)%



// %Tag(CALLBACK)%
void updtPathCallback(const nav_msgs::Path::ConstPtr& newPath)
{

  //int pointsNbr = sizeof(newPath->poses)/sizeof(newPath->poses[0]); 

  int pointsNbr = newPath->poses.size(); 

  ROS_WARN("New target path [%d] points", pointsNbr );

  currentPath.clear();

  for(int i=0; i< pointsNbr; i++ )
  {
    currentPath.push_back(newPath->poses[i]);
  }

  currentDestPoint.x = currentPath[0].pose.position.x;
  currentDestPoint.y = currentPath[0].pose.position.y;

}
// %EndTag(CALLBACK)%



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%


   /*Get parameters*/   
   //geometry_msgs::Twist control;
   //ros::param::get("~vel",control.linear.x);


  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  vel_pub = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
// %EndTag(PUBLISHER)%



  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  /*Husky pose*/
  ros::Subscriber sub = n.subscribe("/odometry/filtered", 10000, guideCallback);
  /*Husky target destination*/
  ros::Subscriber subTarget = n.subscribe("/targetPoint/setpoint", 10000, updtTargetPointCallback);
  /*Husky path*/
  ros::Subscriber subPath = n.subscribe("/targetPath/huskyPath", 10000, updtPathCallback);
// %EndTag(SUBSCRIBER)%



// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  int pathStep =0;
// %Tag(ROS_OK)%
  while (ros::ok())
  {
// %EndTag(ROS_OK)%


  if(calcDistance(huskyIntPosition.x, huskyIntPosition.y, currentDestPoint.x, currentDestPoint.y) < distanceErrorTH)
  {    

    if( pathStep + 1 < currentPath.size() )
    {
      geometry_msgs::Pose tmpPose = currentPath[pathStep+1].pose; 
      currentDestPoint.x = tmpPose.position.x;
      currentDestPoint.y = tmpPose.position.y;
    }
    else
    {
      pathStep=0;
    }

  }

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%

// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%

// %EndTag(ROSCONSOLE)%

// %Tag(PUBLISH)%

// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
  }


  return 0;
}
// %EndTag(FULLTEXT)%

