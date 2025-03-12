#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

void treat_trajectory_request(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

/**
 * @description During the startup process, the turtlebot needs to get an id
 * from the coordinator. It will call a service from the coordinator
 * TODO: call actual service, this is a mock
 */
uint16_t notify_turtle_initial_position() {
  return 3;
  // TODO test if node already exists before trying to create it
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  // TODO NotifyTurtleInitialPosition
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 0;
  request->b = 0;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  return result.get()->sum;
}

/**
 * @description Inform manager that the trajectory is over
 * TODO: call actual service, this is a mock
 */
bool notify_turtle_arrival() {
  return true;
  // TODO test if node already exists before trying to create it
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  // TODO NotifyTurtleArrival
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 0;
  request->b = 0;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  return result.get()->sum;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fetching turtle_id");
  const uint16_t turtle_id = notify_turtle_initial_position();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "turtle_id = %d", turtle_id);

  std::string node_name = "turtlebot_navigation_server_" + std::to_string(turtle_id);
  std::string service_name = "turtlebot_navigation_" + std::to_string(turtle_id);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(node_name);

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("service_name", &treat_trajectory_request);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to perform a trajectory.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
