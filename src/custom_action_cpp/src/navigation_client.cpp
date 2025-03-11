#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "custom_action_interfaces/action/usine_goal_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace custom_action_cpp {
class NavigationActionClient : public rclcpp::Node {
public:
  using UsineGoalPose = custom_action_interfaces::action::UsineGoalPose;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<UsineGoalPose>;

  explicit NavigationActionClient(const rclcpp::NodeOptions &options)
      : Node("navigation_action_client", options) {
    this->client_ptr_ =
        rclcpp_action::create_client<UsineGoalPose>(this, "navigation");

    auto timer_callback_lambda = [this]() { return this->send_goal(); };
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                           timer_callback_lambda);
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = UsineGoalPose::Goal();
    goal_msg.x_goal_pose = 10;
    goal_msg.y_goal_pose = 10;
    goal_msg.theta_goal_pose = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<UsineGoalPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](const GoalHandleNavigation::SharedPtr &goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "Goal accepted by server, waiting for result");
          }
        };

    send_goal_options.feedback_callback =
        [this](GoalHandleNavigation::SharedPtr,
               const std::shared_ptr<const UsineGoalPose::Feedback> feedback) {
          std::stringstream ss;
          ss << "Next number in sequence received: ";
          ss << feedback->x_current_pose << " ";
          RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        };

    send_goal_options.result_callback =
        [this](const GoalHandleNavigation::WrappedResult &result) {
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
          }
          std::stringstream ss;
          ss << "Result received: ";
          ss << result.result->x_final_pose << " ";
          RCLCPP_INFO(this->get_logger(), ss.str().c_str());
          rclcpp::shutdown();
        };
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<UsineGoalPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
}; // class NavigationActionClient

} // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::NavigationActionClient)
