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

#include <ros/ros.h>
#include "pure_pursuit.hpp"
#include <sstream>

namespace ns_pure_pursuit {
// Constructor
PurePursuit::PurePursuit() {
};

// Getters
fsd_common_msgs::ControlCommand PurePursuit::getControlCommand() const { return control_command_; }

// Setters
void PurePursuit::setMaxSpeed(double &max_speed) {
  max_speed_ = max_speed;
}

void PurePursuit::setMap(const fsd_common_msgs::Map &map) {
  map_ = map;
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
  if (velocity_.car_state_dt.x > max_speed_) {
    control_command_.throttle.data = -0.5;
  }
  else {
    control_command_.throttle.data = 0.7;
  }
  control_command_.steering_angle.data = -1.5 + 3* ((double) rand() / (RAND_MAX));
}

}
