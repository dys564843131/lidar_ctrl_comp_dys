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

#include "udp_pkt_ctrl/lidar_udp_raw.h"
#include <sstream>

#define BUFFER_LEN 1500


#define BUFFER_LEN_TATOL 1500
#define ROS_BUFFER_LEN_TATOL 1300


typedef struct
{
    uint32_t trigABI     :3;
    uint32_t ave_int     :5;
    uint32_t v_n         :12;
    uint32_t r_n         :12;
} lidar_PktType;

typedef struct
{
    uint16_t peek0;
    uint16_t index0;
    uint16_t peek1;
    uint16_t index1;
} fpga_PktType;

#define LIDAR_ONCE_PIONT   (300)//just for test ,actual 300 point
#define FPGA_ONCE_PIONT    (100)
//static lidar_PktType lidar_txpkt[LIDAR_ONCE_PIONT];

//static uint8_t  fpga_pkt[LIDAR_ONCE_PIONT * sizeof(fpga_PktType)];

void lidar_format_fpga_to_ros(fpga_PktType *pfpga,lidar_PktType *plidar)
{
    if((pfpga == NULL) || (plidar == NULL))
    {
        printf("pkt err ,check\r\n");
    }

    if(pfpga->index1)
    {
        plidar->r_n = pfpga->index0 + pfpga->index1;
        plidar->v_n = abs(pfpga->index0 - pfpga->index1);
        plidar->ave_int = (pfpga->peek0 + pfpga->peek1) >> 3;
        plidar->trigABI = 0;
    }
    else
    {
        plidar->r_n = pfpga->index0 * 2;
        plidar->v_n = 0;
        plidar->ave_int = pfpga->peek0 *2;
        plidar->trigABI = 0;
    }
    return;
}

void fpga_buf_to_lidar_buf(uint8_t *pfpga_buf,lidar_PktType *plidar_buf,uint32_t pkt_cnt)
{
    uint32_t i;
    fpga_PktType *pfpga;
    lidar_PktType *plidar;

    pfpga = (fpga_PktType *)pfpga_buf;
    plidar = plidar_buf;

    if((pfpga_buf == NULL) || (plidar_buf == NULL))
    {
        printf("buf err ,check\r\n");
    }

    for(i = 0 ; i < pkt_cnt ; i++, pfpga++, plidar++)
    {
        lidar_format_fpga_to_ros(pfpga,plidar);
    }
}

int main(int argc, char **argv)
{

    std::string lidarip;
    int lidarport;
    std::string srvip;
    int srvport;
    int len;

    int sockfd;
    struct sockaddr_in lidarAddr;
    struct sockaddr_in srvAddr;
    socklen_t addr_len = sizeof(lidarAddr);
    unsigned char buffer[BUFFER_LEN];

    unsigned char ros_buffer[ROS_BUFFER_LEN_TATOL];
    int ros_buffer_len = 0;
    static unsigned int curt_ros_point_cnt = 0;


    ros::init(argc, argv, "lidar_udp_srv");
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
        ROS_ERROR("Bind host err,check port");
        exit(1);
    }

    ros::Publisher pub = n.advertise<udp_pkt_ctrl::lidar_udp_raw>("/lidar_pkt_raw", 1000);
    //ros::Rate loop_rate(10);
    int32_t test_cnt = 0;
    uint8_t point_val = 20;
    while (ros::ok())
    {
        udp_pkt_ctrl::lidar_udp_raw msg;
        memset(buffer, 0, BUFFER_LEN);
        len = recvfrom(sockfd, buffer, BUFFER_LEN, 0, (struct sockaddr*)&lidarAddr, &addr_len);
        if(len < 0)
        {
            ROS_WARN("fail receive");
            continue;
        }
        else
        {
            if(len != 800)
            {
                ROS_WARN("packet length:,err");
                continue;
            }

            fpga_buf_to_lidar_buf(buffer, (lidar_PktType *)(ros_buffer + ros_buffer_len), FPGA_ONCE_PIONT);//100 lidar point

            ros_buffer_len += FPGA_ONCE_PIONT * sizeof(lidar_PktType);

            if( (LIDAR_ONCE_PIONT * sizeof(lidar_PktType)) <= ros_buffer_len)
            {
                msg.point_cnt = curt_ros_point_cnt;
                memcpy(&(msg.raw_point[0]),ros_buffer,LIDAR_ONCE_PIONT * sizeof(lidar_PktType));

                pub.publish(msg);

                curt_ros_point_cnt += LIDAR_ONCE_PIONT;//300
                ros_buffer_len %= (LIDAR_ONCE_PIONT * sizeof(lidar_PktType));//1200 byte once
            }

        }

        ros::spinOnce();
    }
    return 0;
}
