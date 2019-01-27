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

#ifndef CONTROL_PURE_PURSUIT_HPP
#define CONTROL_PURE_PURSUIT_HPP
#include "ros/ros.h"

#include <geometry_msgs/Polygon.h>
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
    PurePursuit(ros::NodeHandle& nh);

    // Getters
    fsd_common_msgs::ControlCommand getControlCommand() const;

    // Setters
    void setMaxSpeed(double &max_speed);
    void setCenterLine(const geometry_msgs::Polygon &center_line);
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

    /**
     * Visualize
     */
     void publishMarkers(double x_pos, double y_pos, double x_next, double y_next) const;

    ros::NodeHandle& nh_;

    ros::Publisher pub_closest_point_;

    double speed_p;
    double steering_p;

    geometry_msgs::Polygon          center_line_;
    fsd_common_msgs::CarState       state_;
    fsd_common_msgs::CarStateDt     velocity_;
    fsd_common_msgs::ControlCommand control_command_;

    double max_speed_;
};
}

#endif //CONTROL_PURE_PURSUIT_HPP
