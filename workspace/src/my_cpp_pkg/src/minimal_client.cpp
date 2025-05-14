#include <random>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

/**
 * Client example that periodically calls the server
 */
class MinimalClient : public rclcpp::Node
{
  
public:

  /**
   * Constructor
   */
  MinimalClient() : Node("minimal_client")
  {
    // Create a client object
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>(
      "add_ints");

    // Wait for service
    while (!client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for service...");
    }

    // Seed random number generator with current time
    std::srand(std::time(nullptr));

    // Periodically call method
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000),
      std::bind(&MinimalClient::timer_callback, this));
  }

private:

  /**
   * Send request to server asking it to add two integers
   */
  void timer_callback()
  {
    // Fill out request message
    auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    req->a = std::rand() % 11;
    req->b = std::rand() % 11;

    // Send request to server and set callback
    client_->async_send_request(
      req,
      std::bind(
        &MinimalClient::response_callback,
        this,
        std::placeholders::_1));
  }

  /**
   * Log result when received from server
   */
  void response_callback(
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    auto resp = future.get();
    RCLCPP_INFO(this->get_logger(), "Result: %d", (int)resp->sum);
  }

  // Declare member variables
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * Main entrypoint
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
