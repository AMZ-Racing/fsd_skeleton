/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2018:
     - Sonja Brits <britss@ethz.ch>
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
#include "pure_pursuit_handle.hpp"

// ROS Msgs
#include "geometry_msgs/PolygonStamped.h"

namespace ns_pure_pursuit {

// Constructor
PurePursuitHandle::PurePursuitHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    pure_pursuit_(nodeHandle) {
    ROS_INFO("Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
    pure_pursuit_.setMaxSpeed(max_speed_);
}

// Getters
int PurePursuitHandle::getNodeRate() const { return node_rate_; }

// Methods
void PurePursuitHandle::loadParameters() {
    ROS_INFO("loading handle parameters");
    if (!nodeHandle_.param("max_speed",
                           max_speed_,
                           3.0)) {
        ROS_WARN_STREAM("Did not load max_speed. Standard value is: " << max_speed_);
    }
    if (!nodeHandle_.param<std::string>("slam_map_topic_name",
                                        slam_map_topic_name_,
                                        "/estimation/slam/map")) {
        ROS_WARN_STREAM("Did not load slam_map_topic_name. Standard value is: " << slam_map_topic_name_);
    }
    if (!nodeHandle_.param<std::string>("slam_state_topic_name",
                                        slam_state_topic_name_,
                                        "/estimation/slam/state")) {
        ROS_WARN_STREAM("Did not load slam_state_topic_name. Standard value is: " << slam_state_topic_name_);
    }
    if (!nodeHandle_.param<std::string>("velocity_estimate_topic_name",
                                        velocity_estimate_topic_name_,
                                        "/estimation/velocity_estimation/velocity_estimate")) {
        ROS_WARN_STREAM(
            "Did not load velocity_estimate_topic_name. Standard value is: " << velocity_estimate_topic_name_);
    }
    if (!nodeHandle_.param<std::string>("control_command_topic_name",
                                        control_command_topic_name_,
                                        "/control/pure_pursuit/car_command")) {
        ROS_WARN_STREAM("Did not load control_command_topic_name. Standard value is: " << control_command_topic_name_);
    }
    if (!nodeHandle_.param<std::string>("center_line_topic_name",
                                        center_line_topic_name_,
                                        "/control/pure_pursuit/center_line")) {
        ROS_WARN_STREAM("Did not load center_line_topic_name. Standard value is: " << center_line_topic_name_);
    }
    if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
        ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
    }
}

void PurePursuitHandle::subscribeToTopics() {
    ROS_INFO("subscribe to topics");
    slamMapSubscriber_          =
        nodeHandle_.subscribe(slam_map_topic_name_, 1, &PurePursuitHandle::slamMapCallback, this);
    slamStateSubscriber_        =
        nodeHandle_.subscribe(slam_state_topic_name_, 1, &PurePursuitHandle::slamStateCallback, this);
    velocityEstimateSubscriber_ =
        nodeHandle_.subscribe(velocity_estimate_topic_name_, 1, &PurePursuitHandle::velocityEstimateCallback, this);
}

void PurePursuitHandle::publishToTopics() {
    ROS_INFO("publish to topics");
    controlCommandPublisher_ = nodeHandle_.advertise<fsd_common_msgs::ControlCommand>(control_command_topic_name_, 1);
    centerLinePublisher_     = nodeHandle_.advertise<geometry_msgs::PolygonStamped>(center_line_topic_name_, 1, true);
}

void PurePursuitHandle::run() {
    pure_pursuit_.runAlgorithm();
    sendControlCommand();
}

void PurePursuitHandle::sendControlCommand() {
    control_command_.throttle       = pure_pursuit_.getControlCommand().throttle;
    control_command_.steering_angle = pure_pursuit_.getControlCommand().steering_angle;
    control_command_.header.stamp   = ros::Time::now();
    controlCommandPublisher_.publish(control_command_);
}

void PurePursuitHandle::slamMapCallback(const fsd_common_msgs::Map &map) {
    geometry_msgs::PolygonStamped center_line;
    { // Find Center Line
        center_line.polygon.points.clear();
        for (const auto &yellow: map.cone_yellow) {

            const auto it_blue = std::min_element(map.cone_blue.begin(), map.cone_blue.end(),
                                                  [&](const fsd_common_msgs::Cone &a,
                                                      const fsd_common_msgs::Cone &b) {
                                                      const double da = std::hypot(yellow.position.x - a.position.x,
                                                                                   yellow.position.y - a.position.y);
                                                      const double db = std::hypot(yellow.position.x - b.position.x,
                                                                                   yellow.position.y - b.position.y);

                                                      return da < db;
                                                  });

            geometry_msgs::Point32 p;
            p.x = static_cast<float>((yellow.position.x + it_blue->position.x) / 2.0);
            p.y = static_cast<float>((yellow.position.y + it_blue->position.y) / 2.0);
            p.z = 0.0;
            center_line.polygon.points.push_back(p);
        }
    }

    geometry_msgs::Polygon dense_center_line;
    { // Densify the center line
        const double      precision = 0.2;
        for (unsigned int i         = 1; i < center_line.polygon.points.size(); i++) {
            const double dx = center_line.polygon.points[i].x - center_line.polygon.points[i - 1].x;
            const double dy = center_line.polygon.points[i].y - center_line.polygon.points[i - 1].y;
            const double d  = std::hypot(dx, dy);

            const int         nm_add_points = d / precision;
            for (unsigned int j             = 0; j < nm_add_points; ++j) {
                geometry_msgs::Point32 new_p = center_line.polygon.points[i - 1];
                new_p.x += precision * j * dx / d;
                new_p.y += precision * j * dy / d;
                dense_center_line.points.push_back(new_p);
            }
        }
    }

    center_line.polygon         = dense_center_line;
    center_line.header.frame_id = "map";
    center_line.header.stamp    = ros::Time::now();
    centerLinePublisher_.publish(center_line);

    pure_pursuit_.setCenterLine(dense_center_line);
}

void PurePursuitHandle::slamStateCallback(const fsd_common_msgs::CarState &state) {
    pure_pursuit_.setState(state);
}

void PurePursuitHandle::velocityEstimateCallback(const fsd_common_msgs::CarStateDt &velocity) {
    pure_pursuit_.setVelocity(velocity);
}
}
