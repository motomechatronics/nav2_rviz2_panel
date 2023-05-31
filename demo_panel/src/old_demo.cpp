#include "demo_panel/demo_panel.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("demo_pabel");
  // We create a Rate object of 2Hz
  rclcpp::WallRate loop_rate(2);

    demo_panel::DemoPanel demo;
    
  // Endless loop until Ctrl + C
  while (rclcpp::ok()) {

    demo_panel::DemoPanel demo;

        // RCLCPP_INFO(node->get_logger(), "Help me Obi-Wan Kenobi, you're my
        // only hope");
            rclcpp::spin_some(node);
    // We sleep the needed time to maintain the Rate fixed above
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
// This program creates an endless loop that repeats itself 2 times per second
// (2Hz) until somebody presses Ctrl + C in the Shell