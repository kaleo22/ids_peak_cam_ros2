#ifndef IDS_PEAK_CAM_H
#define IDS_PEAK_CAM_H

#include <iostream>
#include <thread>
#include <cstdlib>
#include <memory>
#include <peak/converters/peak_buffer_converter_ipl.hpp>
#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/camera_info.h>
#include <sensor_msgs/msg/image.h>
#include "std_msgs/msg/string.hpp"

#include <boost/thread/mutex.hpp>

 class ids_cam{

   //ids_cam variables
   std::shared_ptr<peak::core::Device> dev_;
   std::shared_ptr<peak::core::NodeMap> nodeMap_;
   std::shared_ptr<peak::core::DataStream> dStream_;

   std::string serNo_;
   int frameRate_;
   int exposureTime_;

   //ros variables
   std::shared_ptr<rclcpp::Node> nh_;
   image_transport::CameraPublisher img_pub_;
   std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
   std::shared_ptr<image_transport::ImageTransport> it_;

   boost::mutex mutex_;

   void camOpen();

   void camConfig();

   void camAcquisition(sensor_msgs::msg::Image &image);

   void publish(const sensor_msgs::msg::Image::Ptr &image);

   void camClose();

   void camAcquisitionStop();

   public:

     ids_cam(std::shared_ptr<rclcpp::Node> node);

     ~ids_cam();

     void poll();

     void shutdown();

 };

#endif

