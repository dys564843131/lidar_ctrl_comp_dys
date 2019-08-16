roscore
rosrun udp_conf_lidar dynamic_lidar_conf _lidarip:=192.168.199.3 _lidarport:=25001   _hostport:=21001
rosrun rqt_reconfigure rqt_reconfigure

or
roslaunch udp_conf_lidar udp_conf_lidar.launch
