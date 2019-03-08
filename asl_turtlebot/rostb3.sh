export ROS_MASTER_URI=http://sentro.local:11311
export ROS_IP=$(ifconfig enp1s0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')
export ROS_HOSTNAME=$(ifconfig enp1s0| grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')