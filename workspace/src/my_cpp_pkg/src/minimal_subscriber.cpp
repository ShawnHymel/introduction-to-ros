#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

/**
 * Subscriber example that prints messages to the console
 */
class MinimalSubscriber : public rclcpp::Node
{

public:

  /**
   * Constructor (call Node class constructor with node name)
   */
  MinimalSubscriber() : Node("minimal_subscriber")
  {
    // Create a subscription object
    subscription_ = this->create_subscription<example_interfaces::msg::String>(
      "my_topic",
      10,
      std::bind(
        &MinimalSubscriber::listener_callback,
        this,
        std::placeholders::_1));
  }

private:

  /**
   * Prints message to the console
   */
  void listener_callback(const example_interfaces::msg::String & msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg.data.c_str());
  }

  // Declare member variables
  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr 
    subscription_;
};

/**
 * Main entrypoint
 */
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Initialize and run node
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);

  // Cleanup
  rclcpp::shutdown();

  return 0;
}
