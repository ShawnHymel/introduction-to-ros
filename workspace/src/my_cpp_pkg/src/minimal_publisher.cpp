#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

/**
 * Publisher example that periodically sends out a string
 */
class MinimalPublisher : public rclcpp::Node
{

public:

  /**
   * Constructor (call Node class constructor with node name)
   */
  MinimalPublisher() : Node("minimal_publisher")
  {
    // Create a publisher object
    publisher_ = this->create_publisher<example_interfaces::msg::String>(
      "my_topic",
      10);

    // Periodically call method
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MinimalPublisher::timer_callback, this));
    
    // Counter for messages sent
    counter_ = 0;
  }

private:

  /**
   * Publishes simple message to topic
   */
  void timer_callback()
  {
    // Fill out String message
    auto msg = example_interfaces::msg::String();
    msg.data = "Hello, world: " + std::to_string(counter_);

    // Publish message to topic
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());

    // Increment counter
    counter_++;
  }

  // Declare member variables
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t counter_;
};

/**
 * Main entrypoint
 */
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Initialize and run node
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);

  // Cleanup
  rclcpp::shutdown();

  return 0;
}