#include "ids_peak_cam.h"
#include "signal.h"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executor.hpp"


bool sigint_received = false;

void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segementation fualt, stopping camera.\n");
  auto logger = rclcpp::get_logger("SegLog");
  RCLCPP_ERROR(logger, "Segementation faulat, stopping camera.");
  rclcpp::shutdown();
}

void sigint_handler(int signum)
{
  auto logger = rclcpp::get_logger("SIGINT_Log");
  RCLCPP_WARN(logger, "SIGINT RECEIVED!\n");
  sigint_received = true;
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  std::shared_ptr<rclcpp::Node> node =  std::make_shared<rclcpp::Node>("ids_peak_cam_node");
  //auto node = ("ids_peak_cam_node");

  signal(SIGSEGV, &sigsegv_handler);
  ids_cam cam(node);
  signal(SIGINT, &sigint_handler);

  executor.add_node(node);

  while(rclcpp::ok() && !sigint_received)
  {
    cam.poll();
    executor.spin();
  }

  //my_callback_group = NodeBaseInterface::create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //rclcpp::SubscriptionOptions options;
  //options.callback_group = my_callback_group;

  //my_subscription = create_subscription<Int32>("/ids_peak_cam", rclcpp::SensorDataQoS(), callback, options);


  auto logger = rclcpp::get_logger("SD_Log");
  RCLCPP_WARN(logger, "ROS SHUTDOWN!");

  cam.shutdown();

  rclcpp::shutdown();

  return 0;
}

