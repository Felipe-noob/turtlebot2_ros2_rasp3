
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cmath>

//#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

struct GenerationTrajectoireBlock{
    //Inputs
    double p0;
    double pf;
    double v0;
    double vf;
    double t0;
    double tf;
  
    //Outputs
    double q;
    double dq;
  };

struct ReferenceChangeBlock{
    //Inputs
    double x;        // [m]
    double y;        // [m]
    double theta;    // [rad ?]
  
    //Outputs
    double x_head;        // [m]
    double y_head;        // [m]
    double theta_head;    // [rad ?]
};

struct RobotParamater{
    double Km;       // Static Gain [m*s*V]
    double tau;      // Time constant [s]
    double R;        // Turtlebot base radius [m]

    double V_max;
    double W_max;
};

struct GainSpeedBlock{

    // inputs
    double v_desired;
    double w_desired;

    // Gains
    double pre_gain_v = 1.25;    // == 1 / 0.8
    double pre_gain_w = 1.11;    // == 1 / 0.9 

    // outputs
    double v_turtle;
    double w_turtle;

};

struct TurtlebotBlock{

    // input
    double v_turtle;
    double w_turtle;
    double theta;

    //output
    double dx;
    double dy;
    double dtheta;
};

struct TurtlebotReal{

   // input
   double v_turtle;
   double w_turtle;

   //output
   double x;
   double y;
   double theta;
};

struct SaturationBlock
{
    // input
    double ux;
    double uy;

    // saturation condition
    // if | u | > k_sat * V_max, u = V_max * sig(u) * k_sat

    double k_sat ;

    // output

    double ux_sat;
    double uy_sat;

};

struct M_inverseBlock{

    // input
    double ux;
    double uy;
    double theta;

    //output
    double v_desired;
    double w_desired;

};

struct IntegratorBlock
{
    // input
    double dx;
    double dy;
    double dtheta;

    // initial_cond
    double initial_x = 0;
    double initial_y = 0;
    double initial_theta = 0;

    // output
    double x;
    double y;
    double theta;
};

struct CommandBlock{

    // inputs
    double x_ref;
    double y_ref;

    double dx_ref;
    double dy_ref;

    double x_head;
    double y_head;

    // outputs

    double ux;
    double uy;

};

int sign(double x){

    if (x < 0) {
        return -1;
    } 
    else {
        return 1;
    }
   
}

void update_CommandBlock(struct CommandBlock &bloque, double gain_K){

    bloque.ux = gain_K * (bloque.x_ref - bloque.x_head) + bloque.dx_ref;
    bloque.uy = gain_K * (bloque.y_ref - bloque.y_head) + bloque.dy_ref;
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

void update_integrator(struct IntegratorBlock &bloque, double dt){

    static double somme_x = bloque.initial_x; 
    static double somme_y = bloque.initial_y; 
    static double somme_theta = bloque.initial_theta;
    static bool firstCall = true;
    static double last_dx = 0, last_dy = 0, last_dtheta = 0; 

    if(firstCall){
        last_dx = bloque.dx;
        last_dy = bloque.dy;
        last_dtheta = bloque.dtheta;
        firstCall = false;
    }

    bloque.x = (bloque.dx + last_dx)*(dt/2.0f) + somme_x;
    bloque.y = (bloque.dy + last_dy)*(dt/2.0f) + somme_y;
    bloque.theta = (bloque.dtheta + last_dtheta)*(dt/2.0f) + somme_theta;

    last_dx = bloque.dx;
    last_dy = bloque.dy;
    last_dtheta = bloque.dtheta;
    
    somme_x = bloque.x;
    somme_y = bloque.y;
    somme_theta = bloque.theta;
    
}

void update_TurtlebotBlock(struct TurtlebotBlock &bloque){

    bloque.dx = cos(bloque.theta) * bloque.v_turtle;
    bloque.dy = sin(bloque.theta) * bloque.v_turtle;
    bloque.dtheta = bloque.w_turtle;

}

void update_M_inverseBlock(struct M_inverseBlock &bloque,struct RobotParamater &param){
    
    double M_inv[2][2] = {
        {cos(bloque.theta),sin(bloque.theta)},
        {(-1/param.R) * sin(bloque.theta),(1/param.R) * cos(bloque.theta)}
    }; 

    bloque.v_desired = M_inv[0][0] * bloque.ux + M_inv[0][1]*bloque.uy;
    bloque.w_desired = M_inv[1][0] * bloque.ux + M_inv[1][1]*bloque.uy;


}

void update_GainSpeedBlock(struct GainSpeedBlock &bloque){

    bloque.v_turtle = bloque.v_desired * bloque.pre_gain_v;
    bloque.w_turtle = bloque.w_desired * bloque.pre_gain_w;

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

void update_CoordsHead(struct ReferenceChangeBlock &bloque, struct RobotParamater &param){

    bloque.x_head = bloque.x + param.R*cos(bloque.theta);
    bloque.y_head = bloque.y + param.R*sin(bloque.theta);
    bloque.theta_head = bloque.theta;

}

double convert_degrees2radians(double degrees){
   double radians;
    return radians = (degrees*2*M_PI)*360; 
}

int main(int argv, char *argc []){

    // Blocks definition
    struct GenerationTrajectoireBlock block_trajectory_X = {.p0 = 0, .pf = 10, .v0 = 0, .vf = 0, .t0 = 0, .tf = 5, .q = 0, .dq = 0};
    struct GenerationTrajectoireBlock block_trajectory_Y = {.p0 = 0, .pf = 5, .v0 = 0, .vf = 0, .t0 = 0, .tf = 5, .q = 0, .dq = 0};
    struct ReferenceChangeBlock reference_change_block = {};
    struct GainSpeedBlock gain_speed_block = {};
    struct M_inverseBlock inverse_matrix_block = {};
    struct IntegratorBlock itegrator_block = {};
    struct CommandBlock command_block = {};
    

    // Robot's structs
    struct RobotParamater robot_paramaters = {.Km = 1, .tau = 0.025f, .R = 0.17f, .V_max = 0.7f, .W_max = M_PI};
    struct TurtlebotBlock turtlebot2 = {};
  
    double t_current;
    double t;
    double delta = 0.1;

    for(t = 0; t < 20; t = t + delta){

        t_current = t_current + delta; 

        if(t > 10){

            block_trajectory_X.t0 = 0;
            block_trajectory_X.tf = 5;
            block_trajectory_X.p0 = block_trajectory_X.pf;
            block_trajectory_X.pf = 15;

            block_trajectory_Y.t0 = 0;
            block_trajectory_Y.tf = 5;
            block_trajectory_Y.p0 = block_trajectory_Y.pf;
            block_trajectory_Y.pf = 8;
            
        }

        update_TrajectoryOutput(block_trajectory_X,t_current);
        update_TrajectoryOutput(block_trajectory_Y,t_current);

        command_block.x_ref = block_trajectory_X.q;
        command_block.y_ref = block_trajectory_Y.q;
        command_block.dx_ref = block_trajectory_X.dq;
        command_block.dy_ref = block_trajectory_Y.dq;

        command_block.x_head = reference_change_block.x_head;
        command_block.y_head = reference_change_block.y_head;

        update_CommandBlock(command_block,1.0f);

        inverse_matrix_block.ux = command_block.ux;
        inverse_matrix_block.uy = command_block.uy;
        inverse_matrix_block.theta = itegrator_block.theta;

        update_M_inverseBlock(inverse_matrix_block, robot_paramaters);

        //std::cout << inverse_matrix_block.theta << " , "<< inverse_matrix_block.v_desired << " , " << inverse_matrix_block.w_desired  << '\n';

        gain_speed_block.v_desired = inverse_matrix_block.v_desired;
        gain_speed_block.w_desired = inverse_matrix_block.w_desired;

        update_GainSpeedBlock(gain_speed_block);

        //std::cout << turtlebot2.theta << " , "<< gain_speed_block.v_turtle << " , " << gain_speed_block.w_turtle  << '\n';


        turtlebot2.v_turtle = gain_speed_block.v_turtle;
        turtlebot2.w_turtle = gain_speed_block.w_turtle;
        turtlebot2.theta = itegrator_block.theta;

        update_TurtlebotBlock(turtlebot2);

        //std::cout << turtlebot2.dtheta << " , "<< turtlebot2.dx << " , " << turtlebot2.dy  << '\n';

        itegrator_block.dx = turtlebot2.dx;
        itegrator_block.dy = turtlebot2.dy;
        itegrator_block.dtheta = turtlebot2.dtheta;

        update_integrator(itegrator_block, delta);
        
        //std::cout << itegrator_block.x << " , " << itegrator_block.y << " , " << itegrator_block.theta << '\n';

        reference_change_block.x = itegrator_block.x;
        reference_change_block.y = itegrator_block.y;
        reference_change_block.theta = itegrator_block.theta;

        update_CoordsHead(reference_change_block,robot_paramaters);

        std::cout << t_current << " , " << reference_change_block.x_head << " , " << reference_change_block.y_head << " , " << reference_change_block.theta_head << '\n';


    }

    return 0;
}