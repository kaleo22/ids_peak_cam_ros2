#include "ids_peak_cam_4.h"
#include "boost/format.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ids_peak_cam_4");
ids_cam::ids_cam(std::shared_ptr<rclcpp::Node> node):
  nh_(node)
  //cinfo_(new camera_info_manager::CameraInfoManager(nh_)),
  //it_(new image_transport::ImageTransport(nh_)),
  //img_pub_(it_->advertiseCamera("ids_cam/image_raw",1))
{
  rclcpp::Node* rawPtr = nh_.get();
   cinfo_.reset(new camera_info_manager::CameraInfoManager(rawPtr));
   it_.reset(new image_transport::ImageTransport(nh_));
   img_pub_=it_->advertiseCamera("image_raw4",1);
   //,"4104657653"
   nh_->get_parameter("/frameRate", frameRate_);
   nh_->get_parameter("/exposureTime", exposureTime_);
   nh_->get_parameter("/Serial_NO", serNo_);

   camOpen();
   camConfig();

}

ids_cam::~ids_cam()
{
  camAcquisitionStop();
  camClose();
}


void ids_cam::camOpen()
{
  peak::Library::Initialize();
  auto& deviceManager = peak::DeviceManager::Instance();
  deviceManager.Update();
 
  if (deviceManager.Devices().empty())
  {
      auto logger = rclcpp::get_logger("ErrorLog"); 
      RCLCPP_ERROR(logger, "No camera found. Exiting program.");
      peak::Library::Close();
      //std::exit(0);
      rclcpp::shutdown();
      //return 0;
  }

  for (const auto& descriptor: deviceManager.Devices())
  {
    if (descriptor->SerialNumber() == serNo_)
    {
        dev_ = descriptor->OpenDevice(peak::core::DeviceAccessType::Control);
        nodeMap_ = dev_->RemoteDevice()->NodeMaps().at(0);

        break;
    }
  }

  if(dev_ == nullptr)
  {
    auto logger = rclcpp::get_logger("ErrorLog2");
	  RCLCPP_ERROR(logger,"SerNO Wrong. Exiting node.");
     peak::Library::Close();
     rclcpp::shutdown();
  }


  try
  {
            // Open standard data stream
    dStream_ = dev_->DataStreams().at(0)->OpenDataStream();
  }
  catch (const std::exception& e)
  {
            // Open data stream failed
     dev_.reset();
     std::cout << "Failed to open DataStream: " << e.what() << std::endl;
     std::cout << "Exiting program." << std::endl << std::endl;

     peak::Library::Close();
     //return 0;
  }

  //ROS_INFO("%s Opened.", serNo_);
  //std::cout<<serNo<<" opened."<<std::endl;
  if(!cinfo_->setCameraName(serNo_))
  {
    auto logger = rclcpp::get_logger("WS_Log");
    RCLCPP_WARN_STREAM(logger, "["<<serNo_<<"] name not valid"
        <<" for camera_info_manager");
  }
}

void ids_cam::camConfig()
{

  try
  {
      nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector")
          ->SetCurrentEntry("Default");
      nodeMap_->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->Execute();
	    //load default set

      // wait until the UserSetLoad command has been finished
      nodeMap_->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->WaitUntilDone();

  }
  catch (const std::exception&)
  {
      // UserSet is not available, try to disable ExposureStart or FrameStart trigger manually
      std::cout << "Failed to load UserSet Default. Manual freerun configuration." << std::endl;

      try
      {
          nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
              ->SetCurrentEntry("ExposureStart");
          nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
              ->SetCurrentEntry("Off");
      }
      catch (const std::exception&)
      {
          try
          {
              nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
                  ->SetCurrentEntry("FrameStart");
              nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
                  ->SetCurrentEntry("Off");
          }
          catch (const std::exception&)
          {
              // There is no known trigger available, continue anyway.
          }
      }
  }


        // allocate and announce image buffers
  auto payloadSize = nodeMap_->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
  auto bufferCountMax = dStream_->NumBuffersAnnouncedMinRequired();
  for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
  {
      auto buffer = dStream_->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
      dStream_->QueueBuffer(buffer);
  }
   
  // set a frame rate to 10fps (or max value) since some of the trigger cases require a defined frame rate
  auto frameRateMax = nodeMap_->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
         ->Maximum();

  nodeMap_->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
     ->SetValue(std::min(static_cast<double>(frameRate_), frameRateMax));

  auto exposureTimeMax = nodeMap_->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
    ->Maximum();

  nodeMap_->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
     ->SetValue(std::min(static_cast<double>(exposureTime_), exposureTimeMax));


  // Lock critical features to prevent them from changing during acquisition
  nodeMap_->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

  // start acquisition
  dStream_->StartAcquisition(peak::core::AcquisitionStartMode::Default, peak::core::DataStream::INFINITE_NUMBER);
  nodeMap_->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();

	double exposureTime_current = nodeMap_->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->Value();
	double frameRate_current = nodeMap_->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Value();

  auto logger1 = rclcpp::get_logger("exti_log");
  auto logger2 = rclcpp::get_logger("frara_log");

  RCLCPP_INFO(logger1, "CURRENT EXPOSURE TIME: %f", exposureTime_current);
  RCLCPP_INFO(logger2, "CURRENT FRAME RATE: %f", frameRate_current);
}

void ids_cam::camAcquisition(sensor_msgs::msg::Image &image)
{
   auto buffer = dStream_->WaitForFinishedBuffer(5000);
   //auto dev_image = peak::BufferTo<peak::ipl::Image>(buffer);
   auto image_raw = peak::ipl::Image(peak::BufferTo<peak::ipl::Image>(buffer));
   auto dev_image = image_raw.ConvertTo(peak::ipl::PixelFormatName::RGB8, peak::ipl::ConversionMode::Fast);
	 //show the image by opencv 
   //imgae_ = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC1);
   //
   //
   
   //image.height = dev_image.Height();
   //image.width = dev_image.Width();
   //image.step = dev_image.Width();
   
   image.height = 1200;
   image.width = 1920;
   
   image.encoding = sensor_msgs::image_encodings::RGB8;
   image.step = image.width*static_cast<unsigned int>(sensor_msgs::image_encodings::numChannels(image.encoding)) * static_cast<unsigned int>(sensor_msgs::image_encodings::bitDepth(image.encoding))/8;

   int sizeBuffer = static_cast<int>(dev_image.ByteCount());
   image.data.resize(image.height*image.step);

   // Device buffer is being copied into cv::Mat array
   std::memcpy(&image.data[0], dev_image.Data(), static_cast<size_t>(sizeBuffer));

   // queue buffer
   dStream_->QueueBuffer(buffer);
}


void ids_cam::camAcquisitionStop()
{

  // stop acquistion of camera
  try
  {
      dStream_->StopAcquisition(peak::core::AcquisitionStopMode::Default);
  }
  catch (const std::exception&)
  {
      // Some transport layers need no explicit acquisition stop of the datastream when starting its
      // acquisition with a finite number of images. Ignoring Errors due to that TL behavior.

  auto logger = rclcpp::get_logger("Warn_Log3");
  RCLCPP_WARN(logger, "WARNING: Ignoring that TL failed to stop acquisition on datastream.");
  }
  nodeMap_->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();

  // Unlock parameters after acquisition stop
  nodeMap_->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(0);

  // flush and revoke all buffers
  dStream_->Flush(peak::core::DataStreamFlushMode::DiscardAll);
  for (const auto& buffer : dStream_->AnnouncedBuffers())
  {
      dStream_->RevokeBuffer(buffer);
  }
}
    
void ids_cam::camClose()
{
  peak::Library::Close();
}

void ids_cam::publish(const sensor_msgs::msg::Image::Ptr &image)
{
  sensor_msgs::msg::CameraInfo ci = cinfo_->getCameraInfo();

  img_pub_.publish(*image, ci);

}

void ids_cam::poll()
{
  sensor_msgs::msg::Image::Ptr image(new sensor_msgs::msg::Image);
  camAcquisition(*image);
  publish(image);
}

void ids_cam::shutdown()
{
  camAcquisitionStop();
  camClose();
}
