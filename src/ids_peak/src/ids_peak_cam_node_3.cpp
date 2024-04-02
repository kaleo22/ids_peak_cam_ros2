#include "ids_peak_cam_3.h"
#include <signal.h>
#include <rclcpp/rclcpp.hpp>


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
  //auto node = rclcpp::Node("ids_peak_cam_node_3");
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ids_peak_cam_node_3");
  
  signal(SIGSEGV, &sigsegv_handler);
  ids_cam cam(node);
  signal(SIGINT, &sigint_handler);

  while(rclcpp::ok() && !sigint_received)
  {
    cam.poll();
    rclcpp::spin_some(node);
  }

  auto logger = rclcpp::get_logger("SD_Log");
  RCLCPP_WARN(logger, "ROS SHUTDOWN!");

  cam.shutdown();

  return 0;
}

