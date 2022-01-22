/*
 * @Author: jiangxinyu
 * @Date: 2021-08-09 16:17:10
 * @LastEditTime: 2022-01-22 14:09:32
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /360-0366-demo/lcmHandler.cpp
 */
// extern "C" {
//     #include "camerademo.h"
//     #include "calibration.h"
//     #include "DepthMapWrapper.h"
// }

#include "lcmHandler.h"

// extern uint8_t raw12_img[336*1557];
// extern char ir_image[224*114];
// extern char depth_image[224*114*2];
// extern char pcloud_image[224*114*sizeof(pc_pkt_t)];
// extern char depth_data[224*114*sizeof(depth_data_pkt_t)];
// extern unsigned int pt_count;

typedef struct pc_pack{
  float X;
  float Y;
  float Z;
  float c;
}pc_pkt_t;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline int64_t getTimestamp()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)(t.tv_sec * 1000L + t.tv_nsec / 1000000L);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void lcmHandler::pubDepthImage( char* image , const int width_, const int height_,const int size_ , const std::string &type_)
{
    uint16_t * image_ = (uint16_t*)image;
    lcm_sensor_msgs::Image lcm_image;
    auto ir_t  = timeStamp_;
    lcm_ros::Time irTime;
    irTime.sec = int64_t(ir_t);
    irTime.nsec = (ir_t - irTime.sec) * 1e9;
    lcm_image.header.stamp=irTime;
    lcm_image.width = (int32_t)width_;
    lcm_image.height = (int32_t)height_;
    lcm_image.n_data = (int32_t)size_;
    lcm_image.data.resize(lcm_image.n_data);
    lcm_image.encoding = "16UC1";
    // memcpy(lcm_image.data.data(), image_, lcm_image.n_data*sizeof(uint16_t));
    for ( auto i = 0 ; i < size_; i++ )
    {
        lcm_image.data.at(i) = (uint16_t)(image_[i] & 0X1FFF );
    }
    // std::cout << "lcm_image.data.depth = " << lcm_image.data.at(224*57+112) << "\n";
    const std::string  topicName = type_ + "_image";
    _lcm->publish(topicName, &lcm_image);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void lcmHandler::pubIRImage( char* image , const int width_, const int height_,const int size_ , const std::string &type_)
{
    lcm_sensor_msgs::Image lcm_image;
    auto ir_t  = timeStamp_;
    lcm_ros::Time irTime;
    irTime.sec = int64_t(ir_t);
    irTime.nsec = (ir_t - double(irTime.sec)) * 1e9;
    lcm_image.header.stamp=irTime;//时间戳保持和其他话题消息一致
    lcm_image.width = (int32_t)width_;
    lcm_image.height = (int32_t)height_;
    lcm_image.n_data = (int32_t)size_;
    lcm_image.data.resize(lcm_image.n_data);
    lcm_image.encoding = "16UC1";
    for ( auto i = 0 ; i < size_; i++ )
    {
        lcm_image.data.at(i) = (uint16_t)(image[i]);
    }
    const std::string  topicName = type_ + "_image";
    _lcm->publish(topicName, &lcm_image);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void lcmHandler::pubPointCloud(char* pcloud_image ,const int size_ , const std::string &type_ )
{
    pc_pkt_t* pointcloud = (pc_pkt_t*)pcloud_image;
    lcm_sensor_msgs::PointCloud cloud;
    auto time_t  = timeStamp_;
    lcm_ros::Time pcTime;
    pcTime.sec = int64_t(time_t);
    pcTime.nsec = (time_t - double(pcTime.sec)) * 1e9;
    cloud.header.stamp=pcTime;//时间戳保持和其他话题消息一致
    cloud.n_points = size_;
    cloud.header.frame_id = "base_link";
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensity";
    cloud.channels[0].values.resize(size_);
    for (auto i = 0 ; i < size_; i++ )
    {
        lcm_geometry_msgs::Point32 pointTmp;
        pointTmp.x = pointcloud[i].X;
        pointTmp.y = pointcloud[i].Y;
        pointTmp.z = pointcloud[i].Z;
        cloud.points.push_back(pointTmp);
        cloud.channels[0].values[i] = pointcloud[i].c;
    }
    // std::cout << "pointcloud.z = " << pointcloud[224*57+112].Z << "\n";
    const std::string  topicName = type_ ;
    _lcm->publish(topicName, &cloud);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void lcmHandler::getTime()
{
    timeStamp_  = getTimestamp();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////