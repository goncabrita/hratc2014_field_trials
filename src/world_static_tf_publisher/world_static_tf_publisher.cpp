/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Goncalo Cabrita on 21/04/2014
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>

#include <rtk_ros/UTMConverter.h>

sensor_msgs::NavSatFix fix;
bool got_coordinates;

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    fix = *msg;
    got_coordinates = true;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "world_static_tf_publisher");

    ROS_INFO("World static tf publisher");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    got_coordinates = false;

    ros::Subscriber sub = n.subscribe("gps/fix", 50, fixCallback);

    ros::Rate loop(10.0);
    while(ros::ok() && !got_coordinates)
    {
        ros::spinOnce();
        loop.sleep();
    }

    UTMCoordinates utm;
    UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);

    // Generate the transformation from the world frame to the minefield frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(utm.easting, utm.northing, fix.altitude) );
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);

    // Publish the transformation at the desired rate!
    while(ros::ok())
    {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
