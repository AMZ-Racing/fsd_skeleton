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

#ifndef ESTIMATION_SLAM_SLAM_HPP
#define ESTIMATION_SLAM_SLAM_HPP

#include "fsd_common_msgs/ConeDetections.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/Map.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"

namespace ns_slam {

class Slam {

 public:
  // Constructor
  Slam();

  // Getters
  std::vector<fsd_common_msgs::Cone> getMap() const;
  geometry_msgs::Pose2D getState() const;

  // Setters
  void setMaxMapSize(int size);

  /**
   *  initializes the cone map
   */
  void initializeMap();

  /**
   *  initializes the state
   */
  void initializeState();

  /**
   *  updates the cone map
   */
  void updateMap(const fsd_common_msgs::ConeDetections &cones);

  /**
  *  calculates the new car state
  */
  void calculateState(const fsd_common_msgs::CarStateDt &velocity);

  /**
   * calls the other functions in the right order
   */
  void runAlgorithm();

 private:

  std::vector<fsd_common_msgs::Cone> cone_map_;
  geometry_msgs::Pose2D slam_state_;
  int max_map_size_;
};
}

#endif //ESTIMATION_SLAM_SLAM_HPP
