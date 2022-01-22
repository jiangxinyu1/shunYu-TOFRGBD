/*
 * @Author: jiangxinyu
 * @Date: 2021-08-09 16:16:37
 * @LastEditTime: 2022-01-20 11:35:52
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /360-0366-demo/lcmHandler.h
 */
 #ifndef _IMAGES_HANDLER_H_
 #define _IMAGES_HANDLER_H_

#include "lcm/lcm-cpp.hpp"
#include "lcm_std_msgs/Float32MultiArray.hpp"
#include "lcm_msgs/lcm_sensor_msgs/Image.hpp"
#include "lcm_msgs/lcm_sensor_msgs/PointCloud.hpp"
#include "lcm_msgs/lcm_geometry_msgs/Point32.hpp"
#include "lcm_ros/time.h"
#include "lcm_ros/lcm_utils.h"
#include "protocol/protocol.h"
// #include <opencv2/opencv.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline int64_t getTimestamp();

class lcmHandler{

public:
    lcmHandler(lcm::LCM * lcm) : _lcm(lcm) {}

    void pubDepthImage(char* image , const int width_, const int height_,const int size_ , const std::string &type_);
    void pubIRImage(char* image , const int width_, const int height_,const int size_ , const std::string &type_);
    void pubPointCloud(char* pcloud_image ,const int size_ , const std::string &type_ );
    void getTime();

    bool isLcmOk()
    {
        if ( _lcm->handleTimeout(50) < 0 )
        {
            return false;
        }
        return true;
    }

private:
    lcm::LCM * _lcm = nullptr ;
    int64_t timeStamp_;
};

#endif

