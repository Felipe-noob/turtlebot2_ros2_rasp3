#include "coordinator_interface/srv/notify_turtle_arrival.hpp"
#include "coordinator_interface/srv/notify_turtle_initial_position.hpp"
#include "custom_action_interfaces/action/usine_goal_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtle_interface/srv/turtle_move.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include <cstdlib>
#include <iostream>

using UsineGoalPose = custom_action_interfaces::action::UsineGoalPose;
using namespace std::chrono_literals;

std::mutex mutex_in_trajectory;

auto goal_msg = UsineGoalPose::Goal();
enum TurtlePosition_e {
  UNKNOWN,
  INPUT_SIDE1,
  INPUT_SIDE2,
  OUTPUT_SIDE1,
  OUTPUT_SIDE2
};
uint32_t goal_position_id = UNKNOWN;

void treat_trajectory_request(
    const std::shared_ptr<turtle_interface::srv::TurtleMove::Request> request,
    std::shared_ptr<turtle_interface::srv::TurtleMove::Response> response) {
  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
              "Incoming request\nturtle_id: %d"
              " turtle_position: %d",
              request->turtle_id, request->turtle_position);

  response->ack = 0; // TODO see code for OK
  switch (request->turtle_position) {
  case INPUT_SIDE1:
    goal_msg.x_goal_pose = 0;
    goal_msg.y_goal_pose = 0.4;
    goal_msg.theta_goal_pose = 0;
    break;
  case INPUT_SIDE2:
    goal_msg.x_goal_pose = 0;
    goal_msg.y_goal_pose = -0.4;
    goal_msg.theta_goal_pose = 0;
    break;
  case OUTPUT_SIDE1:
    goal_msg.x_goal_pose = 3.6;
    goal_msg.y_goal_pose = 0.4;
    goal_msg.theta_goal_pose = 0;
    break;
  case OUTPUT_SIDE2:
    goal_msg.x_goal_pose = 3.6;
    goal_msg.y_goal_pose = -0.4;
    goal_msg.theta_goal_pose = 0;
    break;
  case UNKNOWN:
  default:
    goal_msg.x_goal_pose = 0;
    goal_msg.y_goal_pose = 0;
    goal_msg.theta_goal_pose = 0;
    response->ack = 1; // TODO see code for error
    break;
  }
  if (response->ack == 0) {
    mutex_in_trajectory.unlock();
    // If it's already in a trajectory, it will quietly ignore the new reference
    // TODO: send error ACK if already in trajectory
  }
  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
              "sending back response: [%ld]", (long int)response->ack);
}

/**
 * @description During the startup process, the turtlebot needs to get an id
 * from the coordinator. It will call a service from the coordinator
 */
uint16_t notify_turtle_initial_position() {
  std::string node_name = "turtlebot_navigation_init";

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared(node_name.c_str());
  rclcpp::Client<coordinator_interface::srv::NotifyTurtleInitialPosition>::
      SharedPtr client = node->create_client<
          coordinator_interface::srv::NotifyTurtleInitialPosition>(
          "get_turtle_id");
  // TODO verify service name from André

  auto request = std::make_shared<
      coordinator_interface::srv::NotifyTurtleInitialPosition::Request>();

  char *env_p = std::getenv("INITIAL_TURTLE_POSITION");
  if (!env_p) {
    throw std::runtime_error("INITIAL_TURTLE_POSITION not defined");
  } else {
    request->turtle_position = std::atoi(env_p);
  }
  RCLCPP_INFO(rclcpp::get_logger(node_name.c_str()),
              "Read initial position = %d", request->turtle_position);

  goal_position_id = request->turtle_position;
  switch (request->turtle_position) {
  case INPUT_SIDE1:
    request->x_turtle = 0;
    request->y_turtle = 0.4;
    break;
  case INPUT_SIDE2:
    request->x_turtle = 0;
    request->y_turtle = -0.4;
    break;
  case OUTPUT_SIDE1:
    request->x_turtle = 3.6;
    request->y_turtle = 0.4;
    break;
  case OUTPUT_SIDE2:
    request->x_turtle = 3.6;
    request->y_turtle = -0.4;
    break;
  case UNKNOWN:
  default:
    request->x_turtle = 0;
    request->y_turtle = 0;
    throw std::runtime_error("INITIAL_TURTLE_POSITION not valid");
    break;
  }

  return 3; // TODO verify service name and remove this return
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger(node_name.c_str()),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger(node_name.c_str()),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger(node_name.c_str()), "Sum: %d",
                result.get()->turtle_id);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(node_name.c_str()),
                 "Failed to call service add_two_ints");
  }

  return result.get()->turtle_id;
}

/**
 * @description Inform manager that the trajectory is over
 */
uint32_t notify_turtle_arrival(uint16_t turtle_id,
                               UsineGoalPose::Result final_pose) {
  return true;
  // TODO Verify actual service name
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<coordinator_interface::srv::NotifyTurtleArrival>::SharedPtr
      client =
          node->create_client<coordinator_interface::srv::NotifyTurtleArrival>(
              "add_two_ints");

  auto request = std::make_shared<
      coordinator_interface::srv::NotifyTurtleArrival::Request>();
  request->turtle_id = turtle_id;
  request->turtle_position = goal_position_id;
  request->x_turtle = final_pose.x_final_pose;
  request->y_turtle = final_pose.y_final_pose;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("turtlebot_navigation_server"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"), "ACK: %d",
                result.get()->ack);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("turtlebot_navigation_server"),
                 "Failed to call service add_two_ints");
  }

  return result.get()->ack;
}

void server(uint16_t turtle_id) {
  std::string node_name =
      "turtlebot_navigation_server_" + std::to_string(turtle_id);
  std::string service_name = "TurtleMove" + std::to_string(turtle_id);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(node_name);

  rclcpp::Service<turtle_interface::srv::TurtleMove>::SharedPtr service =
      node->create_service<turtle_interface::srv::TurtleMove>(
          service_name, &treat_trajectory_request);

  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
              "Ready to perform a trajectory.");

  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
              "Shutdown server thread...");
  rclcpp::shutdown();
}

/**
 * Run the action server without blocking the main thread
 */
void summon_action(uint16_t turtle_id) {
  std::string new_action_name = "navigation_" + std::to_string(turtle_id);
  std::string new_node_name =
      "navigation_action_server_" + std::to_string(turtle_id);

  std::string command =
      "OLDNAME=navigation NEWNAME=" + new_action_name +
      "; "
      "ros2 run custom_action_cpp navigation_server --ros-args "
      "-r __node:=" +
      new_node_name +
      " "
      "-r /$OLDNAME/_action/feedback:=/$NEWNAME/_action/feedback "
      "-r /$OLDNAME/_action/status:=/$NEWNAME/_action/status "
      "-r /$OLDNAME/_action/cancel_goal:=/$NEWNAME/_action/cancel_goal "
      "-r /$OLDNAME/_action/get_result:=/$NEWNAME/_action/get_result "
      "-r /$OLDNAME/_action/send_goal:=/$NEWNAME/_action/send_goal &";

  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"), "Command: %s",
              command.c_str());
  int returnCode = system(command.c_str());
  // Give it time to initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // checking if the command was executed successfully
  if (returnCode == 0) {
    RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
                "Started navigation_server successfully");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("turtlebot_navigation_server"),
                 "Error starting navigation_server");
    throw std::runtime_error("summon_action failed");
  }
}

std::vector<std::string> split(const std::string &s) {
  std::stringstream ss(s);
  std::vector<std::string> words;
  for (std::string w; ss >> w;)
    words.push_back(w);
  return words;
}

UsineGoalPose::Result call_action(uint16_t turtle_id) {
  std::string new_action_name = "navigation_" + std::to_string(turtle_id);
  std::string new_node_name =
      "navigation_action_server_" + std::to_string(turtle_id);

  std::string command =
      "export X_GOAL_POSE=" + std::to_string(goal_msg.x_goal_pose) + " " +
      "Y_GOAL_POSE=" + std::to_string(goal_msg.y_goal_pose) + " " +
      "THETA_GOAL_POSE=" + std::to_string(goal_msg.theta_goal_pose) + " " +
      "TURTLE_ID=" + std::to_string(turtle_id) + "; " +
      "ros2 run custom_action_cpp navigation_client 2>&1";

  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"), "Command: %s",
              command.c_str());

  std::array<char, 256> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"),
                                                pclose);

  if (!pipe) {
    throw std::runtime_error("Error opening pipe!");
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    std::string line(buffer.data());

    // Redirect everything to the terminal
    std::cout << line;

    // Verifica se a linha contém a string desejada
    if (line.find("Result") != std::string::npos) {
      result = line;
    }
  }
  // Separate the line using spaces as a delimitor
  const std::vector fields = split(result);
  const int s = fields.size();

  UsineGoalPose::Result final_pose = UsineGoalPose::Result();
  final_pose.x_final_pose = atof(fields.at(s - 3).c_str());
  final_pose.y_final_pose = atof(fields.at(s - 2).c_str());
  final_pose.theta_final_pose = atof(fields.at(s - 1).c_str());

  std::stringstream ss;
  ss << "Final pose: (";
  ss << final_pose.x_final_pose;
  ss << ", ";
  ss << final_pose.y_final_pose;
  ss << ", ";
  ss << final_pose.theta_final_pose;
  ss << ")";
  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
              ss.str().c_str());
  return final_pose;
}

void shutdown(std::thread &thread_server) {
  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
              "Shutdown rclcpp...");
  rclcpp::shutdown();
  thread_server.join();
  // end action server
  system("pkill -n navigation_s");
  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"), "Exiting...");
}

int main(int argc, char **argv) {
  mutex_in_trajectory.lock();
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
              "Fetching turtle_id");
  const uint16_t turtle_id = notify_turtle_initial_position();
  RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
              "turtle_id = %d", turtle_id);

  std::thread thread_server(server, turtle_id);
  // Wait for thread to come online. I could use a variable for this instead
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  summon_action(turtle_id);

  while (true) {
    mutex_in_trajectory.lock();
    RCLCPP_INFO(rclcpp::get_logger("turtlebot_navigation_server"),
                "locked mutex");

    // Demand trajectory to the robot
    UsineGoalPose::Result final_pose = call_action(turtle_id);

    // send sequest to manager
    notify_turtle_arrival(turtle_id, final_pose);
  }

  shutdown(thread_server);
}
