#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

/**
 * Publisher example that periodically sends out a string
 */
class PublisherWithParams : public rclcpp::Node
{

public:

  /**
   * Constructor (call Node class constructor with node name)
   */
  PublisherWithParams() : Node("publisher_with_params")
  {
    // Declare parameters
    this->declare_parameter("message", "Hello");
    this->declare_parameter("timer_period", 1.0);

    // Get parameters and set to member variables
    message_ = this->get_parameter("message").as_string();
    timer_period_ = this->get_parameter("timer_period").as_double();

    // Configure callback for runtime parameter update
    param_cb_handle_ = this->add_post_set_parameters_callback(
      std::bind(
        &PublisherWithParams::post_parameters_callback,
        this,
        std::placeholders::_1));

    // Create a publisher object
    publisher_ = this->create_publisher<example_interfaces::msg::String>(
      "my_topic",
      10);

    // Periodically call method
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PublisherWithParams::timer_callback, this));
  }

private:

  /**
   * Publishes simple message to topic
   */
  void timer_callback()
  {
    // Fill out String message
    auto msg = example_interfaces::msg::String();
    msg.data = message_;

    // Publish message to topic
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
  }

  /**
   * Set parameter after node started
   */
  void post_parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    // Loop through all parameters
    for (const auto &param : parameters) {

      // Update message
      if (param.get_name() == "message") {
        message_ = param.as_string();
        RCLCPP_INFO(
          this->get_logger(),
          "Set %s to %s",
          param.get_name().c_str(),
          param.as_string().c_str());
      }

      // Update timer period
      else if (param.get_name() == "timer_period") {

        // Update member variable
        timer_period_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(),
          "Set %s to %f",
          param.get_name().c_str(),
          param.as_double());
      
        // Reset timer
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::duration<double>(timer_period_),
          std::bind(&PublisherWithParams::timer_callback, this));
      }

      // Unknown parameter
      else {
        RCLCPP_WARN(
          this->get_logger(),
          "Unknown parameter: %s",
          param.get_name().c_str());
      }
    }
  }

  // Declare member variables
  std::string message_;
  double timer_period_;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    param_cb_handle_;
};

/**
 * Main entrypoint
 */
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Initialize and run node
  auto node = std::make_shared<PublisherWithParams>();
  rclcpp::spin(node);

  // Cleanup
  rclcpp::shutdown();

  return 0;
}