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
#include "velocity_estimator.hpp"
#include <sstream>

namespace ns_velocity_estimation {
    // Constructor
    VelocityEstimator::VelocityEstimator() {
    };

    // Getters
    geometry_msgs::Pose2D VelocityEstimator::getVelocityEstimate() const { return velocity_estimate_;}

    void VelocityEstimator::runAlgorithm() {
      createVelocityEstimate();
    }

    void VelocityEstimator::createVelocityEstimate() {
      // Generate random numbers as velocity estimate
      velocity_estimate_.x = 3* ((double) rand() / (RAND_MAX));
      velocity_estimate_.y = 0.5* ((double) rand() / (RAND_MAX));
      velocity_estimate_.theta = 0.1* ((double) rand() / (RAND_MAX));
    }

}
