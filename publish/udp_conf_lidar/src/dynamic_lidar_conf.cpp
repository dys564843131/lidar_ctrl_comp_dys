#include <ros/ros.h>
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
#include <dynamic_reconfigure/server.h>
#include <udp_conf_lidar/udp_lidarConfig.h>

int sockfd;
struct sockaddr_in lidarAddr;
struct sockaddr_in hostAddr;
socklen_t addr_len = sizeof(lidarAddr);
char buffer[BUFFER_LEN];

#define DEFAULT_lidar_DAC_Freq    (5UL)
#define DEFAULT_ldiar_DAC_Low_V   (-1.0f)
#define DEFAULT_ldiar_DAC_High_V  (1.0f)
#define DEFAULT_lidar_TTL_Freq    (150UL)
#define DEFAULT_lidar_DC_cut      (0UL)
#define DEFAULT_lidar_Threshold   (10UL)
#define LIDAR_CONFIG_ACTION       (0UL)

//#define LIDAR_DAC_TO_INT12(f)        ((uint16_t)(((int16_t)((f) * 4096.0 / 10.0)) & 0x0FFF))

#define LIDAR_DAC_TO_GAIN_INT12(low,high)        ((uint16_t)(((((uint8_t)(((high)-(low)) / 0.2)) << 8) + ((uint8_t)(( (((high)-(low)) / 0.2) -  (((uint8_t)(((high)-(low)) / 0.2)) << 8)) * 256))) & 0x7FFF))

#define LIDAR_DAC_TO_OFFSET_INT12(low,high)        ((uint16_t)(((int16_t)(((high) + (low)) / 2 * 4096.0 / 10.0)) & 0x0FFF))
#define LIDAR_TTL_FREQ_SWITH(f)      ((uint16_t)((30000UL / (f)) & 0x0FFF) - 1)

typedef struct
{
    uint16_t lidar_DAC_Freq;
	uint16_t ldiar_DAC_offset;
    uint16_t ldiar_DAC_gain;
    uint16_t lidar_TTL_Freq;
    uint16_t lidar_DC_cut;
    uint16_t lidar_Threshold;
} lidar_conf_para_type;

#define UDP_SEND_PARM_PKT_LEN (sizeof(lidar_conf_para_type))

lidar_conf_para_type default_lidar_conf = {5,LIDAR_DAC_TO_GAIN_INT12(-1.0 ,1.0),LIDAR_DAC_TO_OFFSET_INT12(-1.0 ,1.0),LIDAR_TTL_FREQ_SWITH(150),0,10};
lidar_conf_para_type last_lidar_conf = {5,LIDAR_DAC_TO_GAIN_INT12(-1.0 ,1.0),LIDAR_DAC_TO_OFFSET_INT12(-1.0 ,1.0),LIDAR_TTL_FREQ_SWITH(150),0,10};

lidar_conf_para_type curt_lidar_conf;

void update_lidar_config(dynamic_lidar_conf::udp_lidarConfig &config)
{
    uint16_t tmpDAC_gain = LIDAR_DAC_TO_GAIN_INT12(config.DAC_Low_V,config.DAC_High_V);
	uint16_t tmpDAC_offset = LIDAR_DAC_TO_OFFSET_INT12(config.DAC_Low_V,config.DAC_High_V);
    uint16_t tmpTTL_Freq = LIDAR_TTL_FREQ_SWITH(config.TTL_Freq);
	
    if(config.DAC_Freq != last_lidar_conf.lidar_DAC_Freq)
    {
        last_lidar_conf.lidar_DAC_Freq = config.DAC_Freq ;
        curt_lidar_conf.lidar_DAC_Freq  = (config.DAC_Freq | 0x8000);
    }

	if(tmpDAC_offset != last_lidar_conf.ldiar_DAC_offset)
    {
        last_lidar_conf.ldiar_DAC_offset = tmpDAC_offset ;
        curt_lidar_conf.ldiar_DAC_offset = (tmpDAC_offset | 0x8000);
    }

    if(tmpDAC_gain != last_lidar_conf.ldiar_DAC_gain)
    {
        last_lidar_conf.ldiar_DAC_gain = tmpDAC_gain ;
        curt_lidar_conf.ldiar_DAC_gain = (tmpDAC_gain | 0x8000);
    }


    if(tmpTTL_Freq != last_lidar_conf.lidar_TTL_Freq)
    {
        last_lidar_conf.lidar_TTL_Freq = tmpTTL_Freq ;
        curt_lidar_conf.lidar_TTL_Freq = (tmpTTL_Freq | 0x8000);
    }


    if(config.DC_cut != last_lidar_conf.lidar_DC_cut)
    {
        last_lidar_conf.lidar_DC_cut = config.DC_cut ;
        curt_lidar_conf.lidar_DC_cut = (config.DC_cut | 0x8000);
    }

    if(config.Threshold != last_lidar_conf.lidar_Threshold)
    {
        last_lidar_conf.lidar_Threshold = config.Threshold ;
        curt_lidar_conf.lidar_Threshold = (config.Threshold | 0x8000);
    }
}

void callback(dynamic_lidar_conf::udp_lidarConfig &config, uint32_t level)
{
    int sendbuflen;
    int n;
	uint16_t tmpDAC_gain = LIDAR_DAC_TO_GAIN_INT12(config.DAC_Low_V,config.DAC_High_V);
	uint16_t tmpDAC_offset = LIDAR_DAC_TO_OFFSET_INT12(config.DAC_Low_V,config.DAC_High_V);
	uint16_t tmpTTL_Freq = LIDAR_TTL_FREQ_SWITH(config.TTL_Freq);

    ROS_INFO("Reconfigure Request: %d %f %f %d %d %d %d %d %04x:level:%d",
             config.DAC_Freq,
             config.DAC_Low_V,
             config.DAC_High_V,
             config.TTL_Freq,
             config.DC_cut,
             config.Threshold,
             config.config_action,
             tmpDAC_offset,
             tmpDAC_gain,level);

	if(config.DAC_Low_V > config.DAC_High_V)
	{
	    ROS_ERROR("DAC_Low_V has to be less than or equal to DAC_High_V");
        std::string err_str ="Notice:DAC_Low_V has to be less than or equal to DAC_High_V !!!!";
        config.config_status = err_str.c_str();
		return;
	}
    
    if(30000 % config.TTL_Freq)//
	{
	    ROS_ERROR("TTL_Freq has to be divisible by 30,000");
        std::string err_str ="TTL_Freq has to be divisible by 30,00 !!!!";
        config.config_status = err_str.c_str();
		return;
	}
    


    if(config.config_action == 0)
    {
        std::string edit_str ="editing......,then update to lidar";
        config.config_status = edit_str.c_str();
        return;//
    }
    else if(config.config_action == 2)
    {
        config.DAC_Freq = DEFAULT_lidar_DAC_Freq;
        config.DAC_Low_V = DEFAULT_ldiar_DAC_Low_V;
        config.DAC_High_V = DEFAULT_ldiar_DAC_High_V;
        config.TTL_Freq = DEFAULT_lidar_TTL_Freq;
        config.DC_cut = DEFAULT_lidar_DC_cut;
        config.Threshold = DEFAULT_lidar_Threshold;
        config.config_action = 0;
        return;
    }
    else if(config.config_action == 1 )
    {
        ROS_INFO("udp sending");
        memset(buffer, 0, BUFFER_LEN);
        
        memcpy(&curt_lidar_conf,&last_lidar_conf,sizeof(last_lidar_conf));//init curt conf buf
        update_lidar_config(config);
        memcpy(buffer,&curt_lidar_conf,UDP_SEND_PARM_PKT_LEN);

        std::string err_str ="waiting lidar response......";
        config.config_status = err_str.c_str();

        n = sendto(sockfd, buffer, UDP_SEND_PARM_PKT_LEN, 0, (struct sockaddr*)&lidarAddr, addr_len);
        if (n == -1)
        {
            ROS_ERROR("upd  send err,please check lidar stats:%d,%ld \r\n",n,strlen(buffer));
        }
        else
        {
            /*
            memset(buffer, 0, BUFFER_LEN);
            n = recvfrom(sockfd, buffer, BUFFER_LEN, 0, (struct sockaddr*)&lidarAddr, &addr_len);
            if (n == -1)
            {
               perror("fail to receive");
            }
            else
            {
               buffer[n] = '\0';
               printf("Received data of my world is: %s\n", buffer);
            }
            std::string lidar_str =" parameters update succeed";
            config.config_status = lidar_str.c_str();
            */
        }
        config.config_action = 0;
    }
    else
    {
        ROS_INFO("check config_action err");
    }
}

int main(int argc, char **argv)
{
    // parameters
    std::string lidarip;
    int lidarport;
    std::string hostip;
    int hostport;

    ros::init(argc, argv, "dynamic_lidar_conf");
    ros::NodeHandle n("~");

    n.param<std::string>("lidarip", lidarip, "192.168.1.9");
    n.param<int>("lidarport", lidarport, 5001);

    //n.param<std::string>("hostip", hostip, "192.168.1.10");
    n.param<int>("hostport", hostport, 25001);


    ROS_INFO("lidar: %s:%d", lidarip.c_str(), lidarport);

    bzero(&lidarAddr, sizeof(lidarAddr));
    lidarAddr.sin_family = AF_INET;
    lidarAddr.sin_port = htons(lidarport);
    inet_pton(AF_INET, lidarip.c_str(), &lidarAddr.sin_addr);

    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket err");
        exit(1);
    }

    bzero(&hostAddr, sizeof(hostAddr));
    hostAddr.sin_family = AF_INET;
    hostAddr.sin_port = htons(hostport);
    //inet_pton(AF_INET, hostip.c_str(), &hostAddr.sin_addr);
    hostAddr.sin_addr.s_addr = htons(INADDR_ANY);
    //ROS_INFO("host: %s:%d", hostip.c_str(), hostport);

    if(bind(sockfd, (struct sockaddr *)&hostAddr, sizeof(hostAddr)) < 0)
    {
        perror("Bind host err,check port");
        exit(1);
    }


    dynamic_reconfigure::Server<dynamic_lidar_conf::udp_lidarConfig> server;
    dynamic_reconfigure::Server<dynamic_lidar_conf::udp_lidarConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("udp lidar conf node");
    ros::spin();
    return 0;
}
