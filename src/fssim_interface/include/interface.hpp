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

#ifndef FSSIM_INTERFACE_INTERFACE_HPP
#define FSSIM_INTERFACE_INTERFACE_HPP

// TF Includes
#include <tf/transform_datatypes.h>

// ROS Messages
#include "nav_msgs/Odometry.h"

// FSD_CAR MSGS
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Mission.h"

// FSSIM Messages
#include "fssim_common/Cmd.h"
#include "fssim_common/State.h"
#include "fssim_common/Track.h"
#include "fssim_common/Mission.h"

namespace fssim {
fssim_common::Cmd getFssimCmd(const fsd_common_msgs::ControlCommand &msg) {
    fssim_common::Cmd cmd;
    cmd.dc    = msg.throttle.data;
    cmd.delta = msg.steering_angle.data;
    return cmd;
}

fssim_common::Mission getFssimMissionFinnished(const fsd_common_msgs::Mission &msg) {
    fssim_common::Mission mis;
    mis.finished = msg.finished;
    mis.mission   = msg.mission;
    return mis;
}

}  // namespace fssim

namespace gotthard {

fsd_common_msgs::CarStateDt getStateDt(const nav_msgs::Odometry &odom) {
    fsd_common_msgs::CarStateDt msg;
    msg.header             = odom.header;
    msg.car_state_dt.x     = odom.twist.twist.linear.x;
    msg.car_state_dt.y     = odom.twist.twist.linear.y;
    msg.car_state_dt.theta = odom.twist.twist.angular.z;
    return msg;
}

fsd_common_msgs::CarStateDt getStateDt(const fssim_common::State &odom) {
    fsd_common_msgs::CarStateDt msg;
    msg.header             = odom.header;
    msg.car_state_dt.x     = odom.vx;
    msg.car_state_dt.y     = odom.vy;
    msg.car_state_dt.theta = odom.r;
    return msg;
}

fsd_common_msgs::CarState getState(const fssim_common::State &odom) {
    fsd_common_msgs::CarState msg;
    msg.header          = odom.header;
    msg.car_state.x     = odom.x;
    msg.car_state.y     = odom.y;
    msg.car_state.theta = odom.yaw;
    msg.car_state_dt    = getStateDt(odom);
    return msg;
}

fsd_common_msgs::Cone getConeFromPoint(const geometry_msgs::Point &p, const std::string &color) {
    fsd_common_msgs::Cone cone;
    cone.color.data = color;
    cone.position.x = p.x;
    cone.position.y = p.y;
    cone.position.z = p.z;
    return cone;
}

fsd_common_msgs::Map getMap(const fssim_common::Track &track) {
    fsd_common_msgs::Map msg;
    msg.header = track.header;
    msg.cone_yellow.clear();
    for (const geometry_msgs::Point &c : track.cones_left) {
        msg.cone_yellow.push_back(getConeFromPoint(c, "yellow"));
    }

    msg.cone_blue.clear();
    for (const geometry_msgs::Point &c : track.cones_right) {
        msg.cone_blue.push_back(getConeFromPoint(c, "blue"));
    }
    return msg;
}

}  // namespace gotthard

#endif //FSSIM_INTERFACE_INTERFACE_HPP
