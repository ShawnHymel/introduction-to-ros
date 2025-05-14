#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/accelerometer.hpp"

/**
 * Subscriber example that prints messages to the console
 */
class AccSubscriber : public rclcpp::Node
{

public:

  /**
   * Constructor (call Node class constructor with node name)
   */
  AccSubscriber() : Node("acc_subscriber")
  {
    // Create a subscription object
    subscription_ = this->create_subscription<my_interfaces::msg::Accelerometer>(
      "my_acc",
      10,
      std::bind(
        &AccSubscriber::listener_callback,
        this,
        std::placeholders::_1));
  }

private:

  /**
   * Prints message to the console
   */
  void listener_callback(const my_interfaces::msg::Accelerometer & msg)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Accelerometer: x=%.2f, y=%.2f, z=%.2f",
      msg.x,
      msg.y,
      msg.z);
  }

  // Declare member variables
  rclcpp::Subscription<my_interfaces::msg::Accelerometer>::SharedPtr 
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
  auto node = std::make_shared<AccSubscriber>();
  rclcpp::spin(node);

  // Cleanup
  rclcpp::shutdown();

  return 0;
}
