#include "common.h"



class camShift_ros{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub_;
    bool mm_to_meters;
    std::string image_topic,depth_topic,cam_info_topic;
    message_filters::Synchronizer<MySyncPolicy> *ts_sync;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;


public:
    camShift_ros(ros::NodeHandle nh_);
    void imageDepthCb(const sensor_msgs::ImageConstPtr& img_msg,const sensor_msgs::ImageConstPtr& depth_msg);
    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg);
};


