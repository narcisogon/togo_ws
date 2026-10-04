/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

/*
File summary:
Provides the ROS 2 executable entry point for the DLIO odometry frontend.
*/

#include "dlio/odom.h"

/*
Summary:
Initializes ROS, constructs DLIO, starts its status output, and spins a
multi-threaded executor so LiDAR and IMU callback groups can progress independently.
*/
int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<dlio::OdomNode>();
  // LiDAR processing may wait briefly for IMU coverage. Use an explicit
  // thread count so container CPU-affinity detection cannot collapse this
  // executor to too few workers and starve the IMU/timer callback groups.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  node->requestShutdown();
  executor.remove_node(node);
  node.reset();

  if (rclcpp::ok()) {rclcpp::shutdown();}

  return 0;

}
