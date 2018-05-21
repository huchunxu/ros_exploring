%Setting ROS_MASTER_URI
setenv('ROS_MASTER_URI','http://192.168.1.202:11311')
%Starting ROS MASTER
rosinit

%Creating ROS publisher handle
chatpub = rospublisher('/talker', 'std_msgs/String');
%This is to create the message definition
msg = rosmessage(chatpub);
%Inserting data to message
msg.Data = 'Hello, From Matlab';
%Sending message to topic
send(chatpub,msg);
%Latching the message on topic
latchpub = rospublisher('/talker', 'IsLatching', true);
