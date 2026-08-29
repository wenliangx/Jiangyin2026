#gnome-terminal
#!/bin/bash 

# kill and new
tmux kill-session -t flag
tmux new-session -s flag -n egov2 -d 

#set mouse on
tmux set-option -g mouse on
# tmux set -g pane-border-status top

#  set windows split:
#  0 |  4  |
#  1 |  5  |
#  2 |  6  |
#  3 |  7  |
tmux split-window -h -t flag:egov2   -p 58
tmux split-window -v -t flag:egov2.0 -p 95
tmux split-window -v -t flag:egov2.1 -p 95
tmux split-window -v -t flag:egov2.2 -p 50
tmux split-window -v -t flag:egov2.3 -p 50

tmux split-window -v -t flag:egov2.5 -p 60 
tmux split-window -v -t flag:egov2.6 -p 60 
tmux split-window -v -t flag:egov2.7 -p 50 



# running roscore
tmux select-pane -t flag:egov2.0 
# tmux send-keys "echo flag | sudo chmod 777 /dev/video0" C-m
tmux send-keys "roscore" C-m

# running ll-slam rviz
tmux select-pane -t flag:egov2.1 
tmux send-keys "cd .." C-m 
# tmux send-keys "cd .." C-m 
tmux send-keys "cd livox_ros_driver2_ws" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "sleep 0.5s" C-m 
tmux send-keys "roslaunch --wait livox_ros_driver2 msg_MID360.launch" C-m 

# running ll-slam rosrun
tmux select-pane -t flag:egov2.2
#tmux send-keys "cd .." C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "sleep 0.5s" C-m 
#tmux send-keys "source /home/flag/ORB_SLAM3_IMU_v0.13/Examples/ROS/ORB_SLAM3_IMU_v0.13/build/devel/setup.bash" C-m 
#tmux send-keys "rosrun ORB_SLAM3_IMU_v0.13 Stereo_IMU_Depth_Color_Gravity_LL_SLAM /home/flag/ORB_SLAM3_IMU_v0.13/Vocabulary/ORBvoc.bin /home/flag/ORB_SLAM3_IMU_v0.13/RealSense_D435i.yaml" C-m 
tmux send-keys "roslaunch ra_lio mapping_mid360.launch" C-m

# running swarm
tmux select-pane -t flag:egov2.3
tmux send-keys "sleep 2s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch --wait ego_planner happy_fly.launch" C-m 

tmux select-pane -t flag:egov2.4
tmux send-keys "sleep 5s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch plane_Det det.launch" C-m 

# running finite state machine
tmux select-pane -t flag:egov2.5
tmux send-keys "sleep 2s" C-m 
tmux send-keys "source devel/setup.bash" C-m  
tmux send-keys "roslaunch --wait fsm_ctrl flight_fsm.launch" C-m

# running egoV2-planner
tmux select-pane -t flag:egov2.6
tmux send-keys "sleep 2s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch --wait fsm_ctrl swarm.launch" C-m 

# running rosbag record
tmux select-pane -t flag:egov2.7
tmux send-keys "sleep 5s" C-m 
tmux send-keys "source devel/setup.bash" C-m
# tmux send-keys "rosbag record /mavros/setpoint_raw/attitude /mavros/local_position/pose /mavros/local_position/velocity_local /mavros/imu/data /super/flag_cmd /super/flag_state /Odometry /nmpc_state /fsm_node/visualization/exp_sfc /fsm_node/visualization/frontend_path /fsm_node/visualization/exp_traj"
# tmux send-keys "rosbag record /nmpc_posref /nmpc_posfdb"
tmux send-keys "rosbag record -O /tmp/sim_log /position_cmd_nmpc /Odometry /path /nmpc_state /mavros/setpoint_raw/attitude /mavros/local_position/pose /mavros/local_position/velocity_local /mavros/imu/data /tf_output /ap_global /ego_planner/flag_state /nmpc_posref /Rcicle_pos /Lcicle_pos mavros/vision_pose/pose /vrpn_client_node/jy0/pose /camera/odom/sample /task2_pose /task3_pose /perc_mode"

tmux select-pane -t flag:egov2.8
tmux send-keys "sleep 5s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch apriltag_ros run_apriltag.launch" C-m
# tmux send-keys "tmux kill-ser" 
# tmux send-keys "rosbag record /mavros/local_position/pose /drone_0_planning/pos_cmd /ego_planner/flag_msg /mavros/setpoint_raw/attitude /nmpc_posref /apriltag_global /target_pose /cloud_registered /Odometry /ap_global "

tmux attach-session -t flag 
