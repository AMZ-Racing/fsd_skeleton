#!/bin/bash

# Sonja Brits <britss@ethz.ch> - FSD Driverless 2019
# Adapted from work by Huub Hendrikx <hhendrik@ethz.ch>

# Apt packages

WSTOOL_PACKAGE="python-wstool"

CHECKINSTALL_PACKAGE="checkinstall"

ROS_PACKAGE=(
    "ros-melodic-desktop-full"
    "ros-melodic-ros-base"
)

FSD_WORKPACKAGES=(
    '1_perception'
    '2_estimation'
    '3_control'
)

BLACKLIST_PACKAGES=''

###################################
# Decide whether to include fssim #
###################################
case $1 in
    -f|--fssim)
    FSSIM="TRUE"
    ;;
esac

printf "FSSIM is..."
if [ -z $FSSIM ]; then
    echo "DISABLED"
    BLACKLIST_PACKAGES='fssim_interface fssim'
else 
    echo "ENABLED"
    FSD_WORKPACKAGES=("${FSD_WORKPACKAGES[@]}" 'fssim_interface')
fi

#####################################
# 			Set paths 				#
#####################################
FSD_DEPENDENCY_FILE="dependencies.rosinstall"

# TODO: check whether cpplint is installed
CPPLINT_PACKAGE="cpplint"

ABSOLUTE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOP_LEVEL_DIR_NAME="$(basename "${ABSOLUTE_PATH}")"
FSD_ROOT_VAR=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)
FSD_ROSDEP_DIR="${ABSOLUTE_PATH}/src/4_continuous_integration/rosdep"
ROSDEP_SOURCES_TARGET="/etc/ros/rosdep/sources.list.d"

red=$'\e[1;31m'
green=$'\e[1;32m'
blue=$'\e[1;34m'
end=$'\e[0m'

printf "\n"
printf "Formula Student Driverless dependency updater\n"
printf "Version 2.0 - 2019"
printf "\n\n"

printf "First checking some things...\n"

#########################################
# 		Check installed packages			#
#########################################
printf "Checking if ROS is installed... "
package_OK=$(dpkg-query -W --showformat='${Status}\n' "${ROS_PACKAGE[0]}"  2> /dev/null|grep "install ok installed")

if [ "" == "$package_OK" ]; then
package_OK=$(dpkg-query -W --showformat='${Status}\n' "${ROS_PACKAGE[1]}"  2> /dev/null|grep "install ok installed")
fi
if [ "" == "$package_OK" ]; then
    printf "[${red}NO${end}]\n"
    printf "Please install the ROS using: 'sudo apt install ros-melodic-desktop-full' or 'sudo apt install ros-melodic-ros-base'\n"
    exit 1
fi
printf "[${green}YES${end}]\n"

printf "Checking if checkinstall is installed... "
package_OK=$(dpkg-query -W --showformat='${Status}\n' "${CHECKINSTALL_PACKAGE}"  2> /dev/null|grep "install ok installed")

if [ "" == "$package_OK" ]; then
    printf "[${red}NO${end}]\n"
    printf "Please install the checkinstall package using: 'sudo apt install checkinstall'\n"
    exit 1
fi

printf "[${green}YES${end}]\n"

#########################################
# 		Add nice aliases to ~/.bashrc 	#
#########################################
if [ -z "$FSD_ROOT" ]; then
    printf "Setting Aliases and Variables..."

    if ! grep -Fxq "export FSD_ROOT=$FSD_ROOT_VAR" ~/.bashrc; then
        printf "\nAdding export FSD_ROOT to ~/.bashrc... "
        echo "" >> ~/.bashrc
        echo "# FSD 2019 Environment variables" >> ~/.bashrc
        echo "export FSD_ROOT=$FSD_ROOT_VAR" >> ~/.bashrc
    fi

    if ! grep -Fxq "# FSD aliases source" ~/.bashrc; then
	FSD_ROOT=${FSD_ROOT_VAR}
        printf "\nAdding alias commands to ~/.bashrc... "
        echo "" >> ~/.bashrc
        echo "source ${FSD_ROOT}/fsd_environment.sh" >> ~/.bashrc
    fi
else
	printf "Aliases and Environment variables has been set... "
fi
printf "[${green}YES${end}]\n"  
printf "ALL CHECKS PASSED [${green}YES${end}]\n"

############################################
# Update the git repositories using wstool #
############################################
read -p "Do you want to get/update the repositories (Y/n)? " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [ -z $REPLY ]; then

    if [ -e "${ABSOLUTE_PATH}/.rosinstall" ]; then
        printf "Rosinstall file found in top level directory! So we are removing it!\n"
        rm ${ABSOLUTE_PATH}/.rosinstall
    fi

	printf "We detected that your are running update_dependencies for the first time!\n"
	INIT_RESULT="$(cd "${ABSOLUTE_PATH}" && wstool init)"
	printf "${INIT_RESULT}\n"

    printf "Merging each workpackage into toplevel dependency list...\n"

    for i in ${FSD_WORKPACKAGES[@]}; do
        FSD_WP_DEPENDENCY_FILE="${ABSOLUTE_PATH}/src/${i}/${FSD_DEPENDENCY_FILE}"
        printf "${blue}\n${i}${end}\n"
        printf "Using dependencies in ${FSD_WP_DEPENDENCY_FILE}\n "
        WS_MERGE_RESULT="$(cd "${ABSOLUTE_PATH}" && wstool merge "${FSD_WP_DEPENDENCY_FILE}")"
        printf "${WS_MERGE_RESULT}"
        printf "\n"
    done

    printf "\n Updating all repositories now...\n"
    cd ${ABSOLUTE_PATH}
    wstool update

fi

###################################################
# 	 Update the binary dependencies using rosdep  #	
###################################################
read -p "Do you want to install/update the projects dependencies (Y/n)? " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [ -z $REPLY ]; then

    # Place our FSD dependencies in the rosdep sources folder
    printf "\n\nCopying FSD's ROS dependency list to rosdep sources folder\n"
    printf "Target folder: ${ROSDEP_SOURCES_TARGET}\n"
    printf "Sudo might ask for your password now!\n"
    COPY_FAILED_MESSAGE="[${red}COPY FAILED: The dependency update might fail now!${end}]\n"
    sudo cp "${FSD_ROSDEP_DIR}/fsd-dependencies.yaml" "${ROSDEP_SOURCES_TARGET}/" || { printf "$COPY_FAILED_MESSAGE" ; }
    sudo cp ${FSD_ROSDEP_DIR}/*.rdmanifest "${ROSDEP_SOURCES_TARGET}/" || { printf "$COPY_FAILED_MESSAGE" ; }
    sudo cp "${FSD_ROSDEP_DIR}/10-fsd-dependencies.list" "${ROSDEP_SOURCES_TARGET}/" || { printf "$COPY_FAILED_MESSAGE" ; }

    sudo apt-get update
    # Update rosdep
    rosdep update

    # Install the dependencies
    rosdep install --from-paths "${ABSOLUTE_PATH}/" -r -i -y || { printf "[${red}ROSDEP FAILED${end}]\n" ;}
    
fi

#################################################
# 			Configure catkin workpace 			#
#################################################
echo "Blacklisting packages: " ${BLACKLIST_PACKAGES}
catkin init
if [ -n "${BLACKLIST_PACKAGES}" ]; then
	catkin config --blacklist ${BLACKLIST_PACKAGES} --cmake-args -DCMAKE_BUILD_TYPE=Release
else
	catkin config --no-blacklist --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

#################################################
#           If fssim enabled update deps        #
#################################################
if [ ! -z $FSSIM ]; then
    printf "Updating FSSIM dependencies..."
    cd src/fssim/
    git pull
    ./update_dependencies.sh
    git lfs pull
    cd ../../
fi
