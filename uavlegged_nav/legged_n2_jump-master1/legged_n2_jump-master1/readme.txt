无人机建图启动命令：
#启动无人机仿真。
roslaunch uav_simulator start.launch
#启动深度相机建图节点。
roslaunch map_manager occupancy_map.launch#这个节点会订阅无人机的深度图和位姿，并发布 2D 栅格图到 /occupancy_map/2D_occupancy_map。
 #查看occupancy map 的可视化
roslaunch map_manager rviz.launch 




人形机器人导航启动命令：
#启动机器人仿真
./simulation.sh
#启动导航节点
roslaunch legged_n2_description n2_navigation.launch 

