#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

/**
 * Server example that adds two integers
 */
class MinimalServer : public rclcpp::Node
{

public:

  /**
   * Constructor
   */
  MinimalServer() : Node("minimal_server")
  {
    // Create a service (server) object
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_ints",
      std::bind(
        &MinimalServer::server_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

  private:

  /**
   * Responds with sum of request integers
   */
  void server_callback(
    const example_interfaces::srv::AddTwoInts::Request::SharedPtr req,
    const example_interfaces::srv::AddTwoInts::Response::SharedPtr resp)
  {
    // Contents of response is automatically sent back to client
    resp->sum = req->a + req->b;

    // Log request
    RCLCPP_INFO(
      this->get_logger(),
      "Received request: a=%d, b=%d",
      (int)req->a,
      (int)req->b);
  }

  // Declare member variables
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

/**
 * Main entrypoint
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
