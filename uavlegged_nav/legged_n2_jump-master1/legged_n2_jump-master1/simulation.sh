export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ROBOT_TYPE=ning
source ./devel/setup.bash
#roslaunch uav_simulator uav_legged_shared_world.launch
roslaunch rl_controllers ac_start.launch