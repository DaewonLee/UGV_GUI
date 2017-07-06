clear all;
rosshutdown;
% Start the ROS master.
master = robotics.ros.Core;
%ROS_MASTER_URI=http://cpr-J100-0104:11311/
%MasterIP = 'http://cpr-J100-0104:11311/'; %Jackal
MasterIP =  '192.168.0.21';
rosinit(MasterIP)
%%
% Create a ROS node, which connects to the master.
node = robotics.ros.Node('/GUI');
%%
% Create a publisher and send string data. The publisher attaches to the 
% node object in the first argument.
pub_state = robotics.ros.Publisher(node, '/jackal_state', 'std_msgs/Int8');
msg_state = rosmessage('std_msgs/Int8');
msg_state.Data = 2;
send(pub_state,msg_state);

pub_waypoint = robotics.ros.Publisher(node, '/jackal_waypoint', 'std_msgs/Float64MultiArray');
msg_waypoint = rosmessage('std_msgs/Float64MultiArray');
msg_waypoint.Data = [39.952078, -75.187924];
send(pub_waypoint,msg_waypoint);




%%
% Clear the publisher and ROS node. Shut down the ROS master.
clear('pub','node')
clear('master')