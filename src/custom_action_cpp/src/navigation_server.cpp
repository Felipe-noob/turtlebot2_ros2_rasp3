#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/usine_goal_pose.hpp"
#include "custom_action_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_action_cpp/visibility_control.h"

namespace custom_action_cpp {
class NavigationActionServer : public rclcpp::Node {
public:
  using UsineGoalPose = custom_action_interfaces::action::UsineGoalPose;
  using GoalHandleUsineGoalPose = rclcpp_action::ServerGoalHandle<UsineGoalPose>;

  CUSTOM_ACTION_CPP_PUBLIC
  explicit NavigationActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("navigation_action_server", options) {
    using namespace std::placeholders;

    auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                              std::shared_ptr<const UsineGoalPose::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %f",
                  goal->x_goal_pose );
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
  rclcpp_action::Server<UsineGoalPose>::SharedPtr action_server_;

  void execute(const std::shared_ptr<GoalHandleUsineGoalPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<UsineGoalPose::Feedback>();
    auto result = std::make_shared<UsineGoalPose::Result>();

    for (int i = 1; (i < 100) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->x_final_pose = 2002;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      // sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->x_final_pose = 9;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };

}; // class NavigationActionServer

} // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::NavigationActionServer)
