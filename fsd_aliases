#!/bin/bash

FSD_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)

alias FSD_source="source ${FSD_ROOT}/fsd_environment.sh"
alias FSD_build="catkin clean --yes && catkin build"
alias FSD_cd="cd ${FSD_ROOT}"

# Missions
alias FSD_launch_trackdrive="roslaunch fsd_common_meta trackdrive.launch"
alias FSD_launch_skidpad="roslaunch fsd_common_meta skidpad.launch"
alias FSD_launch_acceleration="roslaunch fsd_common_meta acceleration.launch"
alias FSD_launch_autox="roslaunch fsd_common_meta autox.launch"
alias FSD_launch_scruti="roslaunch fsd_common_meta scruti.launch"

# RVIZ Launching
alias FSD_rviz_trackdrive="roslaunch fsd_common_meta rviz_trackdrive.launch"
alias FSD_rviz_skidpad="roslaunch fsd_common_meta rviz_skidpad.launch"
alias FSD_rviz_acceleration="roslaunch fsd_common_meta rviz_acceleration.launch"
alias FSD_rviz_autox="roslaunch fsd_common_meta rviz_autox.launch"
alias FSD_rviz_scruti="roslaunch fsd_common_meta rviz_scruti.launch"

# FSSIM
alias FSD_ATS="FSD_source; ${FSD_ROOT}/src/fssim/fssim/scripts/launch.py --config ${FSD_ROOT}/src/fssim_interface/fssim_config/ats_simulation.yaml --output ${HOME}/sim_output/"
