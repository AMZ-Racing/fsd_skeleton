/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2018:
     - Sonja Brits <britss@ethz.ch>

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

#ifndef CONTROL_PURE_PURSUIT_HANDLE_HPP
#define CONTROL_PURE_PURSUIT_HANDLE_HPP

#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "pure_pursuit.hpp"

namespace ns_pure_pursuit {

class PurePursuitHandle {

 public:
  // Constructor
  explicit PurePursuitHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendControlCommand();
//  void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;

  ros::Publisher controlCommandPublisher_;
  ros::Publisher centerLinePublisher_;

  ros::Subscriber slamMapSubscriber_;
  ros::Subscriber slamStateSubscriber_;
  ros::Subscriber velocityEstimateSubscriber_;

  void velocityEstimateCallback(const fsd_common_msgs::CarStateDt &velocity);
  void slamMapCallback(const fsd_common_msgs::Map &map);
  void slamStateCallback(const fsd_common_msgs::CarState &state);

  std::string velocity_estimate_topic_name_;
  std::string slam_map_topic_name_;
  std::string slam_state_topic_name_;
  std::string control_command_topic_name_;
  std::string center_line_topic_name_;

  int node_rate_;
  double max_speed_;

  PurePursuit pure_pursuit_;
  fsd_common_msgs::ControlCommand control_command_;
};
}

#endif //CONTROL_PURE_PURSUIT_HANDLE_HPP
