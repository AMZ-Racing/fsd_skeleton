# Interface for FSSIM

In order to create a universal simulator, FSSIM comunicated with your own framework through this interface.
This package does not contain any logic, it only translated messages from FSSIM into your messages. 
In addition to that it contains launch files which are able to launch only smaller part of your framework (e.g. control since in 
this case FSSIM simulates SLAM and localization as it directly streams pose a map)

* `fssim_config`: contains configurations for FSSIM
* `config`: contains definitions of topics which are translated
* `mission_launch`: contains launch file for the sub-section of your autonomous-pipeline

# How to run FSSIM

Detailed description can be found in .....

### Run FSSIM parallely to your code
If you want to run simulator in one terminal window and be able to test your code in parallel execute
1. `roslaunch fssim_interface fssim.launch`
2. Your pipeline, e.g. `roslaunch control_meta trackdrive.launch`

### Run FSSIM and all pipeline automatically
If you want to run everything with one command. Either add your pipeline launch file to `local_simulation.yaml` under `autonomous_stack:`
and then only execute
1. `roslaunch fssim_interface fssim.launch`

In this way you can also start multiple repetitions: add another line under `repetitions:` in `local_simualtion.yaml`

### Run Automatic Test
If you want to run whole bunch of tests without controlling their progress (no RVIZ), call only `FSD_ATS` what will launch `ats_simulation.yaml` config and stores the loged
files and report to `~/sim_output/` folder.
