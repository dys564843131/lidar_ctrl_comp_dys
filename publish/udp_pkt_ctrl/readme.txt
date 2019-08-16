#run lidar pkt recv and change ,send to ros gui
roslaunch udp_pkt_ctrl udp_ctrl.launch



#record lidar pkt
rosbag record /lidar_pkt_raw

#check lidar pkt info
rostopic bw /lidar_pkt_raw
rostopic echo /lidar_pkt_raw



#replay lidar pkt to ros gui tool
roslaunch udp_pkt_ctrl udp_replay.launch
rosbag play 2019-08-07-09-54-05.bag

rosrun udp_pkt_ctrl lidar_udp_srv

#capture topic to txt
rostopic echo -b 2019-08-07-14-35-34.bag -p /lidar_pkt_raw > test_raw.txt
