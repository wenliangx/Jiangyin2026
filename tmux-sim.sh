#gnome-terminal
#!/bin/bash 

# kill and new
tmux kill-session -t flag
tmux new-session -s flag -n egov2 -d 


#set mouse on
tmux set-option -g mouse on

#  set windows split:
#  0 |  2  |
#  1 |  3  |
tmux split-window -h -t flag:egov2     -p 50
tmux split-window -v -t flag:egov2.0   -p 50
tmux split-window -v -t flag:egov2.2   -p 50
tmux split-window -v -t flag:egov2.3   -p 50

# running roscore
tmux select-pane -t flag:egov2.0 
tmux send-keys "roscore" C-m

# running swarm comm node
tmux select-pane -t flag:egov2.1
tmux send-keys "sleep 1s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch --wait fsm_ctrl swarm.launch" C-m 

# running offboard finite state machine node
tmux select-pane -t flag:egov2.2
tmux send-keys "sleep 1s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch --wait fsm_ctrl single.launch" C-m 


# running egoV2-planner node
tmux select-pane -t flag:egov2.3
tmux send-keys "sleep 2s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
tmux send-keys "roslaunch --wait ego_planner happy_sim.launch" C-m 

tmux select-pane -t flag:egov2.4
tmux send-keys "sleep 2s" C-m 
tmux send-keys "source devel/setup.bash" C-m 
#tmux send-keys "rosbag record /mavros/local_position/pose /drone_0_planning/pos_cmd /ego_planner/flag_msg"


tmux attach-session -t flag 