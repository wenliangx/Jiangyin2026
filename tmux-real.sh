#gnome-terminal
#!/bin/bash 

# kill and new
tmux kill-session -t flag
tmux new-session -s flag -n super -d

#set mouse on
tmux set-option -g mouse on
# tmux set -g pane-border-status top

#  set windows split:
#  0 |  4  |
#  1 |  5  |
#  2 |  6  |
#  3 |  7  |
tmux split-window -h -t flag:super   -p 58
tmux split-window -v -t flag:super.0 -p 95
tmux split-window -v -t flag:super.1 -p 95
tmux split-window -v -t flag:super.2 -p 50
tmux split-window -v -t flag:super.3 -p 50

tmux split-window -v -t flag:super.5 -p 60
tmux split-window -v -t flag:super.6 -p 60
tmux split-window -v -t flag:super.7 -p 50



# running roscore
tmux select-pane -t flag:super.0
# tmux send-keys "echo flag | sudo chmod 777 /dev/video0" C-m
tmux send-keys "roscore" C-m

# running MID360 driver at its typical 10 Hz frame rate
tmux select-pane -t flag:super.1
tmux send-keys "source devel/setup.bash" C-m
tmux send-keys "sleep 0.5s" C-m
tmux send-keys "roslaunch --wait point_lio msg_mid360.launch" C-m

# running ll-slam rosrun
tmux select-pane -t flag:super.2
#tmux send-keys "cd .." C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "sleep 0.5s" C-m 
#tmux send-keys "source /home/flag/ORB_SLAM3_IMU_v0.13/Examples/ROS/ORB_SLAM3_IMU_v0.13/build/devel/setup.bash" C-m 
#tmux send-keys "rosrun ORB_SLAM3_IMU_v0.13 Stereo_IMU_Depth_Color_Gravity_LL_SLAM /home/flag/ORB_SLAM3_IMU_v0.13/Vocabulary/ORBvoc.bin /home/flag/ORB_SLAM3_IMU_v0.13/RealSense_D435i.yaml" C-m 
tmux send-keys "roslaunch point_lio stage1_mid360.launch" C-m

# running SUPER planner and mission
tmux select-pane -t flag:super.3
tmux send-keys "sleep 2s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch --wait mission_planner flag_happy_fly.launch" C-m

tmux select-pane -t flag:super.4
tmux send-keys "sleep 5s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch plane_Det det.launch" C-m 

# running finite state machine
tmux select-pane -t flag:super.5
tmux send-keys "sleep 2s" C-m 
tmux send-keys "source devel/setup.bash" C-m  
tmux send-keys "roslaunch --wait fsm_ctrl single.launch" C-m 

# running user command; Point-LIO bridge replaces px4_estimator
tmux select-pane -t flag:super.6
tmux send-keys "sleep 2s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch --wait fsm_ctrl swarm.launch start_estimator:=false" C-m

# running rosbag record
tmux select-pane -t flag:super.7
tmux send-keys "sleep 5s" C-m 
tmux send-keys "source devel/setup.bash" C-m
# tmux send-keys "rosbag record /mavros/setpoint_raw/attitude /mavros/local_position/pose /mavros/local_position/velocity_local /mavros/imu/data /super/flag_cmd /super/flag_state /Odometry /nmpc_state /fsm_node/visualization/exp_sfc /fsm_node/visualization/frontend_path /fsm_node/visualization/exp_traj"
# tmux send-keys "rosbag record /nmpc_posref /nmpc_posfdb"
tmux send-keys "rosbag record -O /tmp/sim_log /position_cmd_nmpc /point_lio/odometry /pointlio_mavros_bridge/healthy /mavros/odometry/out /Odometry /cloud_registered /path /nmpc_state /mavros/setpoint_raw/attitude /mavros/local_position/pose /mavros/local_position/velocity_local /mavros/imu/data /tf_output /ap_global /super/flag_state /super/flag_cmd /nmpc_posref /Rcicle_pos /Lcicle_pos /camera/odom/sample /task2_pose /task3_pose /perc_mode"

tmux select-pane -t flag:super.8
tmux send-keys "sleep 5s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch apriltag_ros run_apriltag.launch" C-m
# tmux send-keys "tmux kill-ser" 
# tmux send-keys "rosbag record /mavros/local_position/pose /drone_0_planning/pos_cmd /ego_planner/flag_msg /mavros/setpoint_raw/attitude /nmpc_posref /apriltag_global /target_pose /cloud_registered /Odometry /ap_global "

tmux attach-session -t flag 
