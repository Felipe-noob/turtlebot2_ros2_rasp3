#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include <string>
#include <iostream>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_action_interfaces/action/usine_goal_pose.hpp"
#include "custom_action_cpp/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

template <
    class result_t   = std::chrono::milliseconds,
    class clock_t    = std::chrono::steady_clock,
    class duration_t = std::chrono::milliseconds
>
auto since(std::chrono::time_point<clock_t, duration_t> const& start)
{
    return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}

namespace custom_action_cpp {
class NavigationActionServer : public rclcpp::Node {
public:
  using UsineGoalPose = custom_action_interfaces::action::UsineGoalPose;
  using GoalHandleUsineGoalPose =
      rclcpp_action::ServerGoalHandle<UsineGoalPose>;

  CUSTOM_ACTION_CPP_PUBLIC
  explicit NavigationActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("navigation_action_server", options) {
    using namespace std::placeholders;


    // subscribe to odom
    std::string node_name = "odom";

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      node_name, 10, std::bind(&NavigationActionServer::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Listener on topic /%s", node_name.c_str());

    // Publish to /cmd_vel
     publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Navigation Action
    auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                              std::shared_ptr<const UsineGoalPose::Goal> goal) {
      RCLCPP_INFO(this->get_logger(),
                  "Received goal request with order [%f, %f, %f]",
                  goal->x_goal_pose, goal->y_goal_pose, goal->theta_goal_pose);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel =
        [this](const std::shared_ptr<GoalHandleUsineGoalPose> goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handle_accepted =
        [this](const std::shared_ptr<GoalHandleUsineGoalPose> goal_handle) {
          // this needs to return quickly to avoid blocking the executor,
          // so we declare a lambda function to be called inside a new thread
          auto execute_in_thread = [this, goal_handle]() {
            return this->execute(goal_handle);
          };
          std::thread{execute_in_thread}.detach();
        };

    this->action_server_ = rclcpp_action::create_server<UsineGoalPose>(
        this, "navigation", handle_goal, handle_cancel, handle_accepted);

  }

private:
  // Subscription
  double convert_quaternion(double w, double z) const{
    return 2*asin(z);
  }
  mutable std::mutex mutex_cycle;

  mutable geometry_msgs::msg::Point actual_pose = geometry_msgs::msg::Point();
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
      actual_pose.x = msg->pose.pose.position.x;
      actual_pose.y = msg->pose.pose.position.y;
      double w = msg->pose.pose.orientation.w;
      double z = msg->pose.pose.orientation.z;

      actual_pose.z = convert_quaternion(w, z);
      RCLCPP_INFO(this->get_logger(), "Recieved data: (x: %lf, y: %lf, theta: %lf)", actual_pose.x, actual_pose.y, actual_pose.z);

      // allow the calculations to take place
      mutex_cycle.unlock();
    }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void send_command(){
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0;
    message.linear.y = 0;
    message.linear.z = 0;

    message.angular.x = 0;
    message.angular.y = 0;
    message.angular.z = -1;

    RCLCPP_INFO(this->get_logger(), "Publishing to cmd_vel");
    this->publisher_->publish(message);
  }

  // Action
  rclcpp_action::Server<UsineGoalPose>::SharedPtr action_server_;


  void execute(const std::shared_ptr<GoalHandleUsineGoalPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<UsineGoalPose::Feedback>();
    auto result = std::make_shared<UsineGoalPose::Result>();

    // TODO: stop condition
    // TODO: import command from Gabriel
    for (int i = 1; (i < 10) && rclcpp::ok(); ++i) {
      mutex_cycle.lock();
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->x_final_pose = 2002;
        result->y_final_pose = 2002;
        result->theta_final_pose = 2002;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Send command to motors
      send_command();

      // TODO: read data from robot

      // TODO: calculate command for next cycle

      // Send action feedback
      feedback->x_current_pose = actual_pose.x;
      feedback->y_current_pose = actual_pose.y;
      feedback->theta_current_pose = actual_pose.z;

      feedback->x_intermediate_goal_pose = 3;
      feedback->y_intermediate_goal_pose = 3;
      feedback->theta_intermediate_goal_pose = 3;

      feedback->x_speed = 3;
      feedback->y_speed = 3;
      feedback->theta_speed = 3;

      feedback->x_error_pose = 3;
      feedback->y_error_pose = 3;
      feedback->theta_error_pose = 3;

      feedback->x_error_speed = 3;
      feedback->y_error_speed = 3;
      feedback->theta_error_speed = 3;

      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->x_final_pose = 9;
      result->y_final_pose = 8;
      result->theta_final_pose = 7;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };

}; // class NavigationActionServer

} // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::NavigationActionServer)
