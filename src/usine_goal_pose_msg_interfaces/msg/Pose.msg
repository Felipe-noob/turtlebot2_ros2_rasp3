# This is the minimal information we want to pass to our robot.
# A node will recieve this demand in the form of an action, break it down in 
# a trajectory, feed these intermediate points to a node that implements the 
# state feedback control.

# Ideally, we recommend using the standard Navigation2 stack for a more accurate
# trajectory, but there have been some difficulties migrating from ROS1 and
# Ubuntu 16
float64 x
float64 y
float64 theta
