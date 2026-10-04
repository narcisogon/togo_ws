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
Provides the ROS 2 executable entry point for DLIO's optional local map node.
*/

#include "dlio/map.h"

/*
Summary:
Initializes ROS, constructs the map accumulator, and spins its callbacks.
*/
int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<dlio::MapNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();

  if (rclcpp::ok()) {rclcpp::shutdown();}

  return 0;

}
