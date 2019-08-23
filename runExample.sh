set -e

cd catkin
catkin_make opt_control_node
./devel/lib/opt_control/opt_control_node
octave ../ROS/opt_control/src/plotTraj.m
cd ..

