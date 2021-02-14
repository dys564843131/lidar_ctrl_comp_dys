#include "ros/ros.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "aqronos_fft_record/udp_pkt_raw.h"

#include <sstream>

aqronos_fft_record::udp_pkt_raw msg;

int curt_udp_pkt_len = 1144;

#define UDP_RECV_BUFFER_LEN 1500

int main(int argc, char **argv)
{
    std::string lidarip;
    int lidarport;
    std::string srvip;
    int srvport;
    int recv_udp_pkt_len = 0;

    int sockfd;
    struct sockaddr_in lidarAddr;
    struct sockaddr_in srvAddr;
    socklen_t addr_len = sizeof(lidarAddr);
    unsigned char udp_recv_buf[UDP_RECV_BUFFER_LEN];

    ros::init(argc, argv, "udp_pkg_serv_node");
    ros::NodeHandle n("~");
    /*
    n.param<std::string>("lidarip", lidarip, "192.168.199.3");
    n.param<int>("lidarport", lidarport, 5000);
    */
    n.param<std::string>("srvip", srvip, "192.168.199.169");
    n.param<int>("srvport", srvport, 25000);

    ROS_INFO("srv: %s:%d", srvip.c_str(), srvport);

    bzero(&lidarAddr, sizeof(lidarAddr));
    /*lidarAddr.sin_family = AF_INET;
    lidarAddr.sin_port = htons(lidarport);
    inet_pton(AF_INET, lidarip.c_str(), &lidarAddr.sin_addr);
    */
    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        ROS_ERROR("socket err");
        exit(1);
    }

    bzero(&srvAddr, sizeof(srvAddr));
    srvAddr.sin_family = AF_INET;
    srvAddr.sin_port = htons(srvport);
    inet_pton(AF_INET, srvip.c_str(), &srvAddr.sin_addr);

    if(bind(sockfd, (struct sockaddr *)&srvAddr, sizeof(srvAddr)) < 0)
    {
        ROS_ERROR("lidar binde err,lidar status");
        exit(1);
    }

    ros::Publisher pub = n.advertise<aqronos_fft_record::udp_pkt_raw>("/udp_pkt_raw", 1000);

    struct timeval timeOut;
    timeOut.tv_sec = 0;                 //set 1s timeout
    timeOut.tv_usec = 50000;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeOut, sizeof(timeOut)) < 0)
    {
        ROS_ERROR("time out setting failed\n");
        exit(1);
    }

    
    while (ros::ok())
    {
        recv_udp_pkt_len = recvfrom(sockfd, udp_recv_buf, sizeof(udp_recv_buf), 0, (struct sockaddr*)&lidarAddr, &addr_len);
        if(recv_udp_pkt_len < 0)
        {
            continue;
        }
        else
        {
            if(curt_udp_pkt_len != recv_udp_pkt_len)//
            {
                ROS_WARN("pkt_len:%d,err", recv_udp_pkt_len);
                continue;
            }

            msg.lidar_ver = 0x70000;
            msg.udp_pkt_len = curt_udp_pkt_len;
            memcpy(&(msg.udp_raw_data[0]), udp_recv_buf, curt_udp_pkt_len);
            pub.publish(msg);
        }

        ros::spinOnce();
    }
    return 0;
}







