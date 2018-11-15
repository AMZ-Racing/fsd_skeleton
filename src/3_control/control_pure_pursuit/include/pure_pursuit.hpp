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

#ifndef CONTROL_PURE_PURSUIT_HPP
#define CONTROL_PURE_PURSUIT_HPP

#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

namespace ns_pure_pursuit {

class PurePursuit {

 public:
  // Constructor
  PurePursuit();

  // Getters
  fsd_common_msgs::ControlCommand getControlCommand() const;

  // Setters
  void setMaxSpeed(double &max_speed);
  void setMap(const fsd_common_msgs::Map &map);
  void setState(const fsd_common_msgs::CarState &state);
  void setVelocity(const fsd_common_msgs::CarStateDt &velocity);

  /**
   *  creates the cone detections
   */
  void createControlCommand();

  /**
   * calls the other functions in the right order
   */
  void runAlgorithm();

 private:
  fsd_common_msgs::Map map_;
  fsd_common_msgs::CarState state_;
  fsd_common_msgs::CarStateDt velocity_;
  fsd_common_msgs::ControlCommand control_command_;

  double max_speed_;
};
}

#endif //CONTROL_PURE_PURSUIT_HPP
