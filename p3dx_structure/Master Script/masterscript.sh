
#!/bin/bash

cd ~/obstacle_ws
colcon build || { echo "Build failed"; exit 1; }
source install/setup.bash
cd

SESSION="ros_multi"
HOST_IP="192.168.1.100"  # <-- Replace with actual IP

# Start new session and create top-left pane (Pane 0)
PANE0=$(tmux new-session -d -s $SESSION -P -F "#{pane_id}")
tmux send-keys -t $PANE0 "ssh ubuntu@$HOST_IP" C-m
tmux send-keys -t $PANE0 "source /opt/ros/noetic/setup.bash && roscore" C-m

# Split horizontally → Pane 1
PANE1=$(tmux split-window -h -t $PANE0 -P -F "#{pane_id}")
tmux send-keys -t $PANE1 "ssh ubuntu@$HOST_IP" C-m
tmux send-keys -t $PANE1 "source /opt/ros/noetic/setup.bash && rosrun rosaria RosAria" C-m

# Split horizontally → Pane 2
PANE2=$(tmux split-window -h -t $PANE1 -P -F "#{pane_id}")
tmux send-keys -t $PANE2 "ssh ubuntu@$HOST_IP" C-m
tmux send-keys -t $PANE2 "source /opt/ros/noetic/setup.bash && rosrun pioneer rb1_ros1.py" C-m

sleep 3

# Split vertically all 3 to get the second row
PANE3=$(tmux split-window -v -t $PANE0 -P -F "#{pane_id}")
tmux send-keys -t $PANE3 'source /opt/ros/foxy/setup.bash' C-m
tmux send-keys -t $PANE3 'source ~/realsense_ws/install/setup.bash' C-m
tmux send-keys -t $PANE3 'ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true' C-m

PANE4=$(tmux split-window -v -t $PANE1 -P -F "#{pane_id}")
tmux send-keys -t $PANE4 'source /opt/ros/foxy/setup.bash' C-m
tmux send-keys -t $PANE4 'rviz2' C-m

PANE5=$(tmux split-window -v -t $PANE2 -P -F "#{pane_id}")
tmux send-keys -t $PANE5 'source /opt/ros/foxy/setup.bash' C-m
tmux send-keys -t $PANE5 'source ~/obstacle_ws/install/setup.bash' C-m
tmux send-keys -t $PANE5 'ros2 run obstacle_detector obstacle_detection' C-m

sleep 2

# Third row (bottom row of grid)

PANE6=$(tmux split-window -v -t $PANE3 -P -F "#{pane_id}")
tmux send-keys -t $PANE6 'source /opt/ros/foxy/setup.bash' C-m
tmux send-keys -t $PANE6 'cd ~/potential_ws' C-m
tmux send-keys -t $PANE6 'colcon build' C-m
tmux send-keys -t $PANE6 'cd' C-m
tmux send-keys -t $PANE6 'source ~/potential_ws/install/setup.bash' C-m
tmux send-keys -t $PANE6 'ros2 run potential_field_pkg potential_field' C-m

PANE7=$(tmux split-window -v -t $PANE4 -P -F "#{pane_id}")
tmux send-keys -t $PANE7 'source /opt/ros/foxy/setup.bash' C-m
tmux send-keys -t $PANE7 'cd ~/p3dx_ws' C-m
tmux send-keys -t $PANE7 'colcon build' C-m
tmux send-keys -t $PANE7 'source install/setup.bash' C-m
tmux send-keys -t $PANE7 'ros2 run rb2 rb2_ros2' C-m

PANE8=$(tmux split-window -v -t $PANE5 -P -F "#{pane_id}")
tmux send-keys -t $PANE8 'source /opt/ros/foxy/setup.bash' C-m
tmux send-keys -t $PANE8 'cd ~/p3dx_ws' C-m
tmux send-keys -t $PANE8 'colcon build' C-m
tmux send-keys -t $PANE8 'source install/setup.bash' C-m
tmux send-keys -t $PANE8 'ros2 run rb2 keyboard_control' C-m

# Even 3x3 layout
tmux select-layout -t $SESSION tiled

# Attach to session
tmux attach -t $SESSION
