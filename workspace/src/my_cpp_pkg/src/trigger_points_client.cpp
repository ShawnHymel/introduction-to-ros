#include <random>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "my_interfaces/srv/trigger_points.hpp"

/**
 * Client example that periodically calls the server
 */
class TriggerPointsClient : public rclcpp::Node
{
  
public:

  /**
   * Constructor
   */
  TriggerPointsClient() : Node("trigger_points_client")
  {
    // Create a client object
    client_ = this->create_client<my_interfaces::srv::TriggerPoints>(
      "trigger_points");

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
      std::bind(&TriggerPointsClient::timer_callback, this));
  }

private:

  /**
   * Send request to server
   */
  void timer_callback()
  {
    // Fill out request message with random number of requested points
    auto req = std::make_shared<my_interfaces::srv::TriggerPoints::Request>();
    req->num_points = std::rand() % 11;

    // Send request to server and set callback
    client_->async_send_request(
      req,
      std::bind(
        &TriggerPointsClient::response_callback,
        this,
        std::placeholders::_1));
  }

  /**
   * Log result when received from server
   */
  void response_callback(
    rclcpp::Client<my_interfaces::srv::TriggerPoints>::SharedFuture future)
  {
    auto resp = future.get();
    RCLCPP_INFO(
      this->get_logger(),
      "Success: %s",
      resp->success ? "true" : "false");
    for (const auto & point : resp->points)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Point: x=%.2f, y=%.2f, z=%.2f",
        point.x,
        point.y,
        point.z);
    }
  }

  // Declare member variables
  rclcpp::Client<my_interfaces::srv::TriggerPoints>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * Main entrypoint
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TriggerPointsClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
