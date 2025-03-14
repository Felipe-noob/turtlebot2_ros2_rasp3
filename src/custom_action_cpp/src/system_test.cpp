
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "control_system.hpp"
#include "navigation.hpp"
#include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

int main(int argv, char *argc[]) {

  // Blocks definition
  struct GenerationTrajectoireBlock block_trajectory_X = {
      .p0 = 0, .pf = 10, .v0 = 0, .vf = 0, .t0 = 0, .tf = 5, .q = 0, .dq = 0};
  struct GenerationTrajectoireBlock block_trajectory_Y = {
      .p0 = 0, .pf = 5, .v0 = 0, .vf = 0, .t0 = 0, .tf = 5, .q = 0, .dq = 0};
  struct ReferenceChangeBlock reference_change_block = {};
  struct GainSpeedBlock gain_speed_block = {};
  struct M_inverseBlock inverse_matrix_block = {};
  struct IntegratorBlock itegrator_block = {};
  struct CommandBlock command_block = {};

  // Robot's structs
  struct RobotParamater robot_paramaters = {
      .Km = 1, .tau = 0.025f, .R = 0.17f, .V_max = 0.7f, .W_max = M_PI};
  struct TurtlebotBlock turtlebot2 = {};

  double t_current;
  double t;
  double delta = 0.1;

  /*
  auto wps = get_waypoints(1, 3);

  std::cout << "Waypoints intermediarios entre " << 1 << " e " << 3 << ":\n";
  for (const auto& wp : wps) {
      std::cout << "X: " << wp[0] << ", Y: " << wp[1] << "\n";
  }
  */

  for (t = 0; t < 20; t = t + delta) {

    t_current = t_current + delta;

    if (t > 10) {

      block_trajectory_X.t0 = 0;
      block_trajectory_X.tf = 5;
      block_trajectory_X.p0 = block_trajectory_X.pf;
      block_trajectory_X.pf = 15;

      block_trajectory_Y.t0 = 0;
      block_trajectory_Y.tf = 5;
      block_trajectory_Y.p0 = block_trajectory_Y.pf;
      block_trajectory_Y.pf = 8;
    }

    // TODO start
    // To calculate trajectory step

    update_TrajectoryOutput(block_trajectory_X, t_current);
    update_TrajectoryOutput(block_trajectory_Y, t_current);

    // To calculate ux and uy

    update_CommandBlock(command_block, block_trajectory_X, block_trajectory_Y,
                        reference_change_block, 1.0f);

    // To calculate v* and w*

    update_M_inverseBlock(inverse_matrix_block, command_block, robot_paramaters,
                          itegrator_block.theta);

    // To apply the pre compensation gain in v* and w* -> output: v and w

    update_GainSpeedBlock(gain_speed_block, inverse_matrix_block);

    // To simulate the robot (output: dx, dy, dtheta)

    // TODO end
    update_TurtlebotBlock(turtlebot2, gain_speed_block, itegrator_block.theta);

    // To estimate x, y, theta

    update_integrator(itegrator_block, turtlebot2, delta);

    // To simulate the robot head (output: x_head, y_head, theta_head)

    update_CoordsHead(reference_change_block, itegrator_block,
                      robot_paramaters);

    std::cout << t_current << " , " << reference_change_block.x_head << " , "
              << reference_change_block.y_head << " , "
              << reference_change_block.theta_head << '\n';

    // go to the next step
  }

  return 0;
}
