
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cmath>
#include "control_system.hpp"


int sign(double x){

    if (x < 0) {
        return -1;
    } 
    else {
        return 1;
    }
   
}

void update_CommandBlock(struct CommandBlock &command_block,struct GenerationTrajectoireBlock &block_trajectory_X, struct GenerationTrajectoireBlock &block_trajectory_Y , struct ReferenceChangeBlock reference_change_block, double gain_K){

    command_block.x_ref = block_trajectory_X.q;
    command_block.y_ref = block_trajectory_Y.q;
    command_block.dx_ref = block_trajectory_X.dq;
    command_block.dy_ref = block_trajectory_Y.dq;

    command_block.x_head = reference_change_block.x_head;
    command_block.y_head = reference_change_block.y_head;

    command_block.ux = gain_K * (command_block.x_ref - command_block.x_head) + command_block.dx_ref;
    command_block.uy = gain_K * (command_block.y_ref - command_block.y_head) + command_block.dy_ref;
}

void update_SaturationBlock(struct SaturationBlock &bloque, struct RobotParamater &param){

    double ux = bloque.ux;
    double uy = bloque.uy;
    double ux_sat = 0, uy_sat = 0;

    if (abs(ux) > 0.5f * param.V_max){
        ux_sat = param.V_max * sign(ux) * bloque.k_sat;
    } 
    else {
        ux_sat = ux;
    }

    if (abs(uy) > 0.5f * param.V_max){
        uy_sat = param.V_max * sign(uy) * bloque.k_sat;
    } 
    else {
        uy_sat = uy;
    }

    bloque.ux_sat = ux_sat;
    bloque.uy_sat = uy_sat;

}

void update_integrator(struct IntegratorBlock &itegrator_block, struct TurtlebotBlock &turtlebot2 ,double dt){

    // Receive input from robot
    itegrator_block.dx = turtlebot2.dx;
    itegrator_block.dy = turtlebot2.dy;
    itegrator_block.dtheta = turtlebot2.dtheta;


    static double somme_x = itegrator_block.initial_x; 
    static double somme_y = itegrator_block.initial_y; 
    static double somme_theta = itegrator_block.initial_theta;
    static bool firstCall = true;
    static double last_dx = 0, last_dy = 0, last_dtheta = 0; 

    if(firstCall){
        last_dx = itegrator_block.dx;
        last_dy = itegrator_block.dy;
        last_dtheta = itegrator_block.dtheta;
        firstCall = false;
    }

    // Update output

    itegrator_block.x = (itegrator_block.dx + last_dx)*(dt/2.0f) + somme_x;
    itegrator_block.y = (itegrator_block.dy + last_dy)*(dt/2.0f) + somme_y;
    itegrator_block.theta = (itegrator_block.dtheta + last_dtheta)*(dt/2.0f) + somme_theta;

    // Update static variables

    last_dx = itegrator_block.dx;
    last_dy = itegrator_block.dy;
    last_dtheta = itegrator_block.dtheta;
    
    somme_x = itegrator_block.x;
    somme_y = itegrator_block.y;
    somme_theta = itegrator_block.theta;
    
}

void update_TurtlebotBlock(struct TurtlebotBlock &turtlebot2, struct GainSpeedBlock &gain_speed_block, double theta){

    turtlebot2.v_turtle = gain_speed_block.v_turtle;
    turtlebot2.w_turtle = gain_speed_block.w_turtle;
    turtlebot2.theta = theta;

    turtlebot2.dx = cos(turtlebot2.theta) * turtlebot2.v_turtle;
    turtlebot2.dy = sin(turtlebot2.theta) * turtlebot2.v_turtle;
    turtlebot2.dtheta = turtlebot2.w_turtle;

}

void update_M_inverseBlock(struct M_inverseBlock &inverse_matrix_block,struct CommandBlock &command_block, struct RobotParamater &param, double theta){
    
    inverse_matrix_block.ux = command_block.ux;
    inverse_matrix_block.uy = command_block.uy;
    inverse_matrix_block.theta = theta;

    double M_inv[2][2] = {
        {cos(inverse_matrix_block.theta),sin(inverse_matrix_block.theta)},
        {(-1/param.R) * sin(inverse_matrix_block.theta),(1/param.R) * cos(inverse_matrix_block.theta)}
    }; 

    inverse_matrix_block.v_desired = M_inv[0][0] * inverse_matrix_block.ux + M_inv[0][1]*inverse_matrix_block.uy;
    inverse_matrix_block.w_desired = M_inv[1][0] * inverse_matrix_block.ux + M_inv[1][1]*inverse_matrix_block.uy;


}

void update_GainSpeedBlock(struct GainSpeedBlock &gain_speed_block, struct M_inverseBlock &inverse_matrix_block){

    gain_speed_block.v_desired = inverse_matrix_block.v_desired;
    gain_speed_block.w_desired = inverse_matrix_block.w_desired;

    gain_speed_block.v_turtle = gain_speed_block.v_desired * gain_speed_block.pre_gain_v;
    gain_speed_block.w_turtle = gain_speed_block.w_desired * gain_speed_block.pre_gain_w;

}

void update_TrajectoryOutput(struct GenerationTrajectoireBlock &bloque, double time_current){

    static double a[4];
    double lambda;

    a[3] = ((bloque.vf - bloque.v0)*(bloque.tf - bloque.t0)) - (2*(bloque.pf - bloque.p0)-(2*bloque.v0*(bloque.tf - bloque.t0)));
    a[2] = (bloque.pf - bloque.p0) - bloque.v0*(bloque.tf - bloque.t0) - a[3];
    a[1] = bloque.v0*(bloque.tf - bloque.t0);
    a[0] = bloque.p0;

    lambda = (time_current - bloque.t0)/(bloque.tf - bloque.t0);
    double lambda_2 = lambda*lambda;
    double lambda_3 = lambda * lambda_2;

    if(time_current < bloque.tf){
        bloque.q = a[0] + a[1]*lambda + a[2]*lambda_2 + a[3]*lambda_3;
        bloque.dq = (a[1] + 2*a[2]*lambda + 3*a[3]*lambda_2) / (bloque.tf - bloque.t0);
    }
    else{
        bloque.q = bloque.pf;
        bloque.dq = bloque.vf;
    }
}

void update_CoordsHead(struct ReferenceChangeBlock &reference_change_block, struct IntegratorBlock itegrator_block,struct RobotParamater &param){

    // Receive inputs from integrator_block
    reference_change_block.x = itegrator_block.x;
    reference_change_block.y = itegrator_block.y;
    reference_change_block.theta = itegrator_block.theta;

    // Update output
    reference_change_block.x_head = reference_change_block.x + param.R*cos(reference_change_block.theta);
    reference_change_block.y_head = reference_change_block.y + param.R*sin(reference_change_block.theta);
    reference_change_block.theta_head = reference_change_block.theta;

}

double convert_degrees2radians(double degrees){
   double radians;
    return radians = (degrees*2*M_PI)*360; 
}
