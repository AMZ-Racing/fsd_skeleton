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
#include "cone_detector.hpp"
#include <sstream>

namespace ns_cone_detector {
    // Constructor
    ConeDetector::ConeDetector() {
    };

    // Getters
    fsd_common_msgs::ConeDetections ConeDetector::getConeDetections() const { return coneDetections_;}

    void ConeDetector::runAlgorithm() {
      createConeDetections();
    }

    void ConeDetector::createConeDetections() {
      std::vector<fsd_common_msgs::Cone> cones;
      fsd_common_msgs::Cone cone;
      // Create 3 random blue cones
      for (int i = 0; i < 3; i++) {
        cone.position.x = 10 * ((double) rand() / (RAND_MAX));
        cone.position.y = 10 * ((double) rand() / (RAND_MAX));
        cone.color.data = "b";
        cones.push_back(cone);
      }
      coneDetections_.cone_detections = cones;
    }

}
