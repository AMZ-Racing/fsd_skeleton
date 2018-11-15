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

#ifndef ESTIMATION_VELOCITY_ESTIMATION_VELOCITY_ESTIMATOR_HPP
#define ESTIMATION_VELOCITY_ESTIMATION_VELOCITY_ESTIMATOR_HPP

#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"

namespace ns_velocity_estimation {

    class VelocityEstimator {

    public:
        // Constructor
        VelocityEstimator();

        // Getters
        geometry_msgs::Pose2D getVelocityEstimate() const;

        /**
         *  creates the cone detections
         */
        void createVelocityEstimate();

        /**
         * calls the other functions in the right order
         */
        void runAlgorithm();

    private:

      geometry_msgs::Pose2D velocity_estimate_;
    };
}

#endif //ESTIMATION_VELOCITY_ESTIMATION_VELOCITY_ESTIMATOR_HPP
