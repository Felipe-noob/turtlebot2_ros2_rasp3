#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include <string>
#include <iostream>
#include <sstream>

#include "custom_action_interfaces/action/usine_goal_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_action_cpp/visibility_control.h"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/log.hpp"

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
    subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "myrosout", 10, std::bind(&NavigationActionServer::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Listener on topic /topic"); // TODO print real node

    // Publish to /cmd_vel
     publisher_ = this->create_publisher<std_msgs::msg::String>("myrosout2", 10);
    // auto timer_callback =
    //   [this]() -> void {
    //     auto message = std_msgs::msg::String();
    //     message.data = "Hello, world! " + std::to_string(this->count_++);
    //     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //     this->publisher_->publish(message);
    //   };
    // timer_ = this->create_wall_timer(500ms, timer_callback);

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
  mutable uint8_t sensor_data = 0;
  void topic_callback(const rcl_interfaces::msg::Log::SharedPtr msg) const
    {
      // TODO read real data
      sensor_data = msg->level;
      RCLCPP_INFO(this->get_logger(), "I heard: level=%d, name='%s'", msg->level, msg->name.c_str());
    }
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;

  // Publisher
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  void send_command(){
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(this->count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
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
      auto start = std::chrono::steady_clock::now();
      feedback->x_current_pose = 3;
      feedback->y_current_pose = 3;
      feedback->theta_current_pose = 3;
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
      std::cout << "Elapsed(ms)=" << since(start).count() << std::endl;
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->x_final_pose = 9;
      result->y_final_pose = 9;
      result->theta_final_pose = 9;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };

}; // class NavigationActionServer

} // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::NavigationActionServer)
