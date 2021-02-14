#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <dynamic_reconfigure/server.h>
#include <aqronos_fft_record/udppkt_writeConfig.h>
#include "aqronos_fft_record/udp_pkt_raw.h"

aqronos_fft_record::udppkt_writeConfig curt_conf;

#define MAX_POINTS_PER_UDP_PACKET 100

#pragma pack(1)

typedef struct AqronosPacketHeader_t
{
    /* Identify Lidar model, Ensure correct packet info */
    uint32_t magic_number;

    /* Size (bytes) of header */
    uint8_t data_offset;

    /* Size (bytes) of data points */
    uint8_t data_size;

    /* Number of data points */
    uint16_t num_points;

    /* Specify LIDAR identity in multi-lidar environment */
    uint32_t serial_number;

    /* Number of data points sent so far */
    uint32_t frame_counter;

    /* Point to start a new frame on. If larger than num_points,
     * no new lidar frame is in this packet */
    uint16_t frame_reset_point;

    uint16_t reserved;

    /* GPS updated timestamp using system clock */
    uint32_t timestamp;

    uint32_t timestamp_h;
    /* lidar point phi min val*/
    int32_t phi_min;
    /* lidar point phi max val*/
    int32_t phi_max;
    /* lidar point theta min val*/
    int32_t theta_min;
    /* lidar point theta max val*/
    int32_t theta_max;
} AqronosPacketHeader;


typedef struct _udp_fft_dataraw_
{
    uint8_t res_0;
    uint8_t res_1;
    uint8_t res_2;
    uint8_t index_h;
    uint8_t index_l;
    uint8_t data2_h;
    uint8_t data2_m;
    uint8_t data2_l;
    uint8_t data1_h;
    uint8_t data1_m;
    uint8_t data1_l;
}dataRawType;



typedef struct udpfftPacket_t
{
    AqronosPacketHeader header;
    dataRawType data[MAX_POINTS_PER_UDP_PACKET];
} udp_FFT_Packet;



typedef struct _udp_adc_dataraw_
{
    uint8_t res_0;
    uint8_t index_h;
    uint8_t index_l;
    uint8_t adc_data_1_h;
    uint8_t adc_data_1_l;
    uint8_t adc_data_2_h;
    uint8_t adc_data_2_l;
    uint8_t adc_data_3_h;
    uint8_t adc_data_3_l;
    uint8_t adc_data_4_h;
    uint8_t adc_data_4_l;
}udp_adc_dataRawType;

typedef struct adc_data_t
{
    uint8_t res_0;
    uint16_t index;
    int16_t adc_data_1;
    int16_t adc_data_2;
    int16_t adc_data_3;
    int16_t adc_data_4;

    inline adc_data_t(udp_adc_dataRawType *pv8)
    {
        this->res_0 = pv8->res_0;
	    this->index = (pv8->index_h << 8) + pv8->index_l;
		this->adc_data_1 = (pv8->adc_data_1_h << 8) + (pv8->adc_data_1_l);
		if(this->adc_data_1 & 0x2000)
		{
			this->adc_data_1 |= 0xc000;
		}
        this->adc_data_2 = (pv8->adc_data_2_h << 8) + (pv8->adc_data_2_l);
        if(this->adc_data_2 & 0x2000)
		{
			this->adc_data_2 |= 0xc000;
		}
        this->adc_data_3 = (pv8->adc_data_3_h << 8) + (pv8->adc_data_3_l);
        if(this->adc_data_3 & 0x2000)
		{
			this->adc_data_3 |= 0xc000;
		}
        this->adc_data_4 = (pv8->adc_data_4_h << 8) + (pv8->adc_data_4_l);
        if(this->adc_data_4 & 0x2000)
		{
			this->adc_data_4 |= 0xc000;
		}
    }
}adc_dataType;


typedef struct udpdac_Packet_t
{
    AqronosPacketHeader header;
    udp_adc_dataRawType data[MAX_POINTS_PER_UDP_PACKET];
} udp_ADC_Packet;

#pragma pack()


std::ofstream udp_data_out;


void callback(aqronos_fft_record::udppkt_writeConfig &config, uint32_t level)
{
    if(udp_data_out)
    {
        udp_data_out.close();
    }

    if(config.write_enable)
    {
        struct timeval tv;
        struct timezone tz;
        gettimeofday (&tv, &tz);
        std::stringstream ssfilename;
        ssfilename << "udppkt_" << config.file_name << "_" << tv.tv_sec << tv.tv_usec << ".txt";
        udp_data_out.open( ssfilename.str() );
        
        if(udp_data_out)
        {
            udp_data_out << "record start time:" << tv.tv_sec << tv.tv_usec  << std::endl;
            udp_data_out.flush();
        }
		
		if(config.record_datatype == 0)
		{
			ROS_WARN("start ADC data record to file");
		}
		else if(config.record_datatype == 1)
		{
			ROS_WARN("start FFT data record to file");
		}
        
    }
    
    curt_conf = config;
    
}

void udp_pkg_Callback(const aqronos_fft_record::udp_pkt_raw::ConstPtr& msg)
{
    if(!udp_data_out)
    {
        return;
    }
	if(curt_conf.write_enable)//bypass
	{
		if(curt_conf.record_datatype == 0)//adc data
		{
			uint16_t index = 0;
			uint32_t data2 = 0;
			uint32_t data1 = 0;
			udp_ADC_Packet * pcurtpkt = (udp_ADC_Packet *)&(msg->udp_raw_data[0]);
			udp_adc_dataRawType * pcurt = &(pcurtpkt->data[0]);

			for(int i = 0; i < MAX_POINTS_PER_UDP_PACKET; pcurt++, i++)
			{
			    adc_dataType pcurtpoint(pcurt);
			    
				udp_data_out << pcurtpoint.index       << "," << pcurtpoint.adc_data_1 << std::endl;
				udp_data_out << (pcurtpoint.index + 1) << "," << pcurtpoint.adc_data_2 << std::endl;
				udp_data_out << (pcurtpoint.index + 2) << "," << pcurtpoint.adc_data_3 << std::endl;
				udp_data_out << (pcurtpoint.index + 3) << "," << pcurtpoint.adc_data_4 << std::endl;
			}

			udp_data_out.flush();
		}
		else if(curt_conf.record_datatype == 1) //fft data
		{
			uint16_t index = 0;
			uint32_t data2 = 0;
			uint32_t data1 = 0;
			udp_FFT_Packet * pcurtpkt = (udp_FFT_Packet *)&(msg->udp_raw_data[0]);
			dataRawType * pcurt = &(pcurtpkt->data[0]);

			for(int i = 0; i < MAX_POINTS_PER_UDP_PACKET; pcurt++, i++)
			{
				index = pcurt->index_l | (pcurt->index_h << 8);
				data2 = (pcurt->data2_h << 16)| (pcurt->data2_m << 8) | pcurt->data2_l;
				data1 = (pcurt->data1_h << 16)| (pcurt->data1_m << 8) | pcurt->data1_l;
				udp_data_out << index << "," << data2 << "," << data1  << std::endl;
			}

			udp_data_out.flush();
		}
	}
	

}



int main (int argc, char **argv)
{
    ros::init (argc, argv, "udp_pkt_write_to_file");

    ros::NodeHandle nh;

    ros::Subscriber bat_sub = nh.subscribe("/udp_pkt_raw", 1000, udp_pkg_Callback);

    dynamic_reconfigure::Server<aqronos_fft_record::udppkt_writeConfig> server;
    dynamic_reconfigure::Server<aqronos_fft_record::udppkt_writeConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}

