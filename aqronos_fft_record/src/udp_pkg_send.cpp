#include "ros/ros.h"
#include "aqronos_fft_record/udp_pkt_raw.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>


#define UDP_SEND_BUF_LEN 1500


int sockfd;
struct sockaddr_in hostAddr;
socklen_t addr_len = sizeof(hostAddr);
char udp_send_buf[UDP_SEND_BUF_LEN];

void lidar_pkg_send_msg_Callback(const aqronos_fft_record::udp_pkt_raw::ConstPtr& msg)
{
    memcpy(udp_send_buf, &(msg->udp_raw_data[0]), msg->udp_pkt_len);
    int send_len = sendto(sockfd, udp_send_buf, msg->udp_pkt_len, 0, (struct sockaddr*)&hostAddr, addr_len);
    if (send_len < 0)
    {
        ;//printf("upd send err:%d\r\n", n);
    }
    else
    {
        ;
    }
}

int main(int argc, char **argv)
{
    std::string hostip;
    int hostport;
    ros::init(argc, argv, "udp_pkg_send_node");
    ros::NodeHandle n("~");
    n.param<std::string>("hostip", hostip, "127.0.0.1");
    n.param<int>("hostport", hostport, 25000);

    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket err");
        exit(1);
    }

    bzero(&hostAddr, sizeof(hostAddr));
    hostAddr.sin_family = AF_INET;
    hostAddr.sin_port = htons(hostport);
    inet_pton(AF_INET, hostip.c_str(), &hostAddr.sin_addr);
    ROS_INFO("host: %s:%d", hostip.c_str(), hostport);

    ros::Subscriber sub = n.subscribe("/udp_pkt_raw", 100, lidar_pkg_send_msg_Callback);
    ros::spin();
    return 0;
}
