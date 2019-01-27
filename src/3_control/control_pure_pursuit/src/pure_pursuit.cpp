/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2018:
     - Sonja Brits  <britss@ethz.ch>
     - Juraj Kabzan <kabzanj@gmail.com>
     
    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "pure_pursuit.hpp"
#include <sstream>

// ROS Msgs
#include "visualization_msgs/MarkerArray.h"

namespace ns_pure_pursuit {
// Constructor
PurePursuit::PurePursuit(ros::NodeHandle &nh) : nh_(nh) {
    pub_closest_point_ = nh.advertise<visualization_msgs::MarkerArray>("/control/pure_pursuit/marker", 1);

    if (!nh.param<double>("controller/speed/p", speed_p, 0.01)) {
        ROS_WARN_STREAM("Did not load controller/speed/p. Standard value is: " << 0.01);
    }
    if (!nh.param<double>("controller/steering/p", steering_p, 0.01)) {
        ROS_WARN_STREAM("Did not load controller/steering/p. Standard value is: " << 0.01);
    }
};

// Getters
fsd_common_msgs::ControlCommand PurePursuit::getControlCommand() const { return control_command_; }

// Setters
void PurePursuit::setMaxSpeed(double &max_speed) {
    max_speed_ = max_speed;
}

void PurePursuit::setCenterLine(const geometry_msgs::Polygon &center_line) {
    center_line_ = center_line;
}

void PurePursuit::setState(const fsd_common_msgs::CarState &state) {
    state_ = state;
}

void PurePursuit::setVelocity(const fsd_common_msgs::CarStateDt &velocity) {
    velocity_ = velocity;
}

void PurePursuit::runAlgorithm() {
    createControlCommand();
}

void PurePursuit::createControlCommand() {

    if (center_line_.points.empty()) {
        control_command_.throttle.data       = static_cast<float>(-1.0);
        control_command_.steering_angle.data = 0.0;
        return;
    }

    const auto             it_center_line = std::min_element(center_line_.points.begin(), center_line_.points.end(),
                                                             [&](const geometry_msgs::Point32 &a,
                                                                 const geometry_msgs::Point32 &b) {
                                                                 const double da = std::hypot(state_.car_state.x - a.x,
                                                                                              state_.car_state.y - a.y);
                                                                 const double db = std::hypot(state_.car_state.x - b.x,
                                                                                              state_.car_state.y - b.y);

                                                                 return da < db;
                                                             });
    const auto             i_center_line  = std::distance(center_line_.points.begin(), it_center_line);
    const auto             size           = center_line_.points.size();
    const auto             i_next         = (i_center_line + 10) % size;
    geometry_msgs::Point32 next_point = center_line_.points[i_next];

    { // Steering Control
        const double beta_est = control_command_.steering_angle.data * 0.5;
        const double eta      = std::atan2(next_point.y - state_.car_state.y, next_point.x - state_.car_state.x)
                                - (state_.car_state.theta + beta_est);
        const double length   = std::hypot(next_point.y - state_.car_state.y, next_point.x - state_.car_state.x);
        control_command_.steering_angle.data = static_cast<float>(steering_p * std::atan(2.0 / length * std::sin(eta)));

    }
    { // Speed Controller
        const double vel = std::hypot(state_.car_state_dt.car_state_dt.x, state_.car_state_dt.car_state_dt.y);
        control_command_.throttle.data = static_cast<float>(speed_p * (max_speed_ - vel));
    }

    // Visualize
    publishMarkers(it_center_line->x, it_center_line->y, next_point.x, next_point.y);

}

void PurePursuit::publishMarkers(double x_pos, double y_pos, double x_next, double y_next) const{
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker marker;
    marker.color.r            = 1.0;
    marker.color.a            = 1.0;
    marker.pose.position.x    = x_pos;
    marker.pose.position.y    = y_pos;
    marker.pose.orientation.w = 1.0;
    marker.type               = visualization_msgs::Marker::SPHERE;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.id                 = 0;
    marker.scale.x            = 0.5;
    marker.scale.y            = 0.5;
    marker.scale.z            = 0.5;
    marker.header.stamp       = ros::Time::now();
    marker.header.frame_id    = "map";
    markers.markers.push_back(marker);

    marker.pose.position.x = x_next;
    marker.pose.position.y = y_next;
    marker.color.b         = 1.0;
    marker.id              = 1;
    markers.markers.push_back(marker);

    pub_closest_point_.publish(markers);
}

}
