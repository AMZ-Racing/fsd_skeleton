/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// ROS Includes
#include "ros/ros.h"

// TF
#include "tf/transform_broadcaster.h"

#include "interface.hpp"

// FSSIM
static ros::Subscriber sub_fssim_odom;
static ros::Subscriber sub_fssim_res;
static ros::Subscriber sub_fssim_track;

static ros::Publisher pub_fssim_cmd;
static ros::Publisher pub_fssim_mission_finnished;

// FSD_CAR
static ros::Subscriber sub_fsd_car_command;
static ros::Subscriber sub_fsd_mission_finnished;

static ros::Publisher pub_fsd_vel;
static ros::Publisher pub_fsd_state;
static ros::Publisher pub_fsd_map;
static ros::Publisher pub_gotthard_res;

static bool                     tf_base_link = false;
static std::string              fsd_vehicle;
static std::string              origin;
static tf::TransformBroadcaster *br = nullptr;

void callbackFssimOdom(const fssim_common::State::ConstPtr &msg) {
    pub_fsd_vel.publish(gotthard::getStateDt(*msg));
    pub_fsd_state.publish(gotthard::getState(*msg));

    if (tf_base_link) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, msg->yaw);
        transform.setRotation(q);
        br->sendTransform(tf::StampedTransform(transform, msg->header.stamp, origin, fsd_vehicle));
    }
}

void callbackFssimTrack(const fssim_common::Track::ConstPtr &msg) {
    pub_fsd_map.publish(gotthard::getMap(*msg));
}

void callbackFsdCmd(const fsd_common_msgs::ControlCommand::ConstPtr &msg) {
    pub_fssim_cmd.publish(fssim::getFssimCmd(*msg));
}

void callbackFsdMissionFinnished(const fsd_common_msgs::Mission::ConstPtr &msg) {
    pub_fssim_mission_finnished.publish(fssim::getFssimMissionFinnished(*msg));
}

template<class Type>
Type getParam(const ros::NodeHandle &nh, const std::string &name) {
    Type       val;
    const bool success = nh.getParam(name, val);
    assert(success && "PARAMETER DOES NOT EXIST");
    return val;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fssim_interface");
    ros::NodeHandle n("~");

    br = new tf::TransformBroadcaster();

    sub_fsd_car_command = n.subscribe(getParam<std::string>(n, "fsd/cmd"), 1, callbackFsdCmd);
    sub_fsd_mission_finnished =
        n.subscribe(getParam<std::string>(n, "fsd/mission_finnished"), 1, callbackFsdMissionFinnished);

    pub_fsd_vel   = n.advertise<fsd_common_msgs::CarStateDt>(getParam<std::string>(n, "fsd/vel"), 1);
    pub_fsd_state = n.advertise<fsd_common_msgs::CarState>(getParam<std::string>(n, "fsd/state"), 1);
    pub_fsd_map   = n.advertise<fsd_common_msgs::Map>(getParam<std::string>(n, "fsd/map"), 1, true);

    sub_fssim_odom  = n.subscribe(getParam<std::string>(n, "fssim/topic_odom"), 1, callbackFssimOdom);
    sub_fssim_track = n.subscribe(getParam<std::string>(n, "fssim/track"), 1, callbackFssimTrack);

    pub_fssim_cmd = n.advertise<fssim_common::Cmd>(getParam<std::string>(n, "fssim/cmd"), 1);
    pub_fssim_mission_finnished =
        n.advertise<fssim_common::Mission>(getParam<std::string>(n, "fssim/mission_finnished"), 1);

    tf_base_link = getParam<bool>(n, "fsd/tf/publish_car_base_link");
    origin       = getParam<std::string>(n, "fsd/tf/origin");
    fsd_vehicle  = getParam<std::string>(n, "fsd/tf/fsd_car_base_link");

    ros::spin();
    return 0;
}
