#include "ros/ros.h"
#include "udp_pkt_ctrl/lidar_udp_raw.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#define BUFFER_LEN 1500
#define LIDAR_PIONT_CNT_LEN (4)
#define LIDAR_RAW_LEN (1200)
int sockfd;
struct sockaddr_in hostAddr;
socklen_t addr_len = sizeof(hostAddr);
char buffer[BUFFER_LEN];

void messageCallback(const udp_pkt_ctrl::lidar_udp_raw::ConstPtr& msg)
{
    static int sendbuflen = LIDAR_PIONT_CNT_LEN + LIDAR_RAW_LEN;
    int n;
    ROS_INFO("I heard: [%d] [%d]", msg->point_cnt, msg->raw_point[0]);
    memcpy(buffer,&(msg->point_cnt),sizeof(uint32_t));
    memcpy(buffer + sizeof(uint32_t),&(msg->raw_point),LIDAR_RAW_LEN);

    n = sendto(sockfd, buffer, sendbuflen, 0, (struct sockaddr*)&hostAddr, addr_len);
    if (n == -1)
    {
        printf("upd  send err,please check lidar stats:%d,%ld \r\n",n,strlen(buffer));
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
    ros::init(argc, argv, "lidar_udp_send");
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


    ros::Subscriber sub = n.subscribe("/lidar_pkt_raw", 1000, messageCallback);
    ros::spin();
    return 0;
}
