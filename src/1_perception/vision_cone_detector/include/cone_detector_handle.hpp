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

#ifndef PERCEPTION_VISION_CONE_DETECTOR_HANDLE_HPP
#define PERCEPTION_VISION_CONE_DETECTOR_HANDLE_HPP

#include "fsd_common_msgs/ConeDetections.h"
#include "cone_detector.hpp"

namespace ns_cone_detector {

class ConeDetectorHandle {

 public:
  // Constructor
  ConeDetectorHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendConeDetections();
//  void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Publisher coneDetectionsPublisher;

  std::string cone_detections_topic_name_;
  int node_rate_;

  ConeDetector coneDetector_;
  fsd_common_msgs::ConeDetections cone_detections_;
};
}

#endif //PERCEPTION_VISION_CONE_DETECTOR_HANDLE_HPP
