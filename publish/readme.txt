#ROS node 
ROS package for adjust formatting, and save data,just for debug

### Create a static IP Address
The Ethernet interface name should be something like *enp30s0* or *enp0s31f6* and is found with:
```bash
ls /sys/class/net/
```
To set the static ip address of the ethernet interface *enp30s0* to 192.168.1.10, run:
```bash
sudo ifconfig enp30s0 192.168.1.10
```

## Installation
```bash
mkdir -p ~/catkin_ws_aq/src
cd ~/catkin_ws_aq/src
catkin_init_workspace
git clone https://github.com/dys564843131/lidar_ctrl_comp_dys.git
cd ..
catkin_make
echo "source ~/catkin_ws_aq/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

## run 
roslaunch udp_pkt_ctrl udp_ctrl.launch



######Compilation problems
sudo chmod +x ~/catkin_ws_aq/src/lidar_ctrl_comp_dys/publish/udp_conf_lidar/cfg/udp_lidar.cfg

######check lidar pkt info
rostopic bw /lidar_pkt_raw
rostopic echo /lidar_pkt_raw

######ROS ip

then ROS_Server-feature_gui.zip readme.md
tip:
./run.sh host:=127.0.0.1 port:=21000