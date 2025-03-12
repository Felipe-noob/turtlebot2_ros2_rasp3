
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <math.h>

//#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

struct GenerationTrajectoireBlock{
    //Inputs
    float p0;
    float pf;
    float v0;
    float vf;
    float t0;
    float tf;
  
    //Outputs
    float q;
    float dq;
  };

struct ReferenceChangeBlock{
    //Inputs
    float x;        // [m]
    float y;        // [m]
    float theta;    // [rad ?]
  
    //Outputs
    float x_head;        // [m]
    float y_head;        // [m]
    float theta_head;    // [rad ?]
  };

struct RobotParamater{
    float Km;       // Static Gain [m*s*V]
    float tau;      // Time constant [s]
    float R;        // Turtlebot base radius [m]
};

struct SpeedConverterBlock{

    // inputs
    float v_rigth;  //  [m/s]
    float v_left;   //  [m/s]
    
    // outputs
    float v;        // [m/s]
    float w;        // [rad/s]

};

struct GainSpeedBlock{

    // inputs
    float v_desired;
    float w_desired;

    // Gains
    float pre_gain_v = 1.25;    // == 1 / 0.8
    float pre_gain_w = 1.11;    // == 1 / 0.9 

    // outputs
    float v_turtle;
    float w_turtle;

};

struct TurtlebotBlock{

    // input
    float v_turtle;
    float w_turtle;
    float theta;

    //output
    float dx;
    float dy;
    float dtheta;
};

struct M_inverseBlock{

    // input
    float ux;
    float uy;
    float theta;

    //output
    float v_desired;
    float w_desired;

};

struct IntegratorBlock
{
    // input
    float dx;
    float dy;
    float dtheta;

    // initial_cond
    float initial_x = 0;
    float initial_y = 0;
    float initial_theta = 0;

    // output
    float x;
    float y;
    float theta;
};


void calcule_integrator(struct IntegratorBlock &bloque, float dt){

    static float somme_x = bloque.initial_x; 
    static float somme_y = bloque.initial_y; 
    static float somme_theta = bloque.initial_theta;
    static bool firstCall = true;
    static float last_dx = 0, last_dy = 0, last_dtheta = 0; 

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

    bloque.dx = cos(bloque.theta) * bloque.v_turtle * 0.8;
    bloque.dy = sin(bloque.theta) * bloque.v_turtle * 0.8;
    bloque.dtheta = bloque.w_turtle * 0.9;

}

void update_M_inverseBlock(struct M_inverseBlock &bloque,struct RobotParamater &param){
    
    float M_inv[2][2] = {
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

void update_TrajectoryOutput(struct GenerationTrajectoireBlock &bloque, float time_current){

    static float a[4];
    float lambda;

    a[3] = ((bloque.vf - bloque.v0)*(bloque.tf - bloque.t0)) - (2*(bloque.pf - bloque.p0)-(2*bloque.v0*(bloque.tf - bloque.t0)));
    a[2] = (bloque.pf - bloque.p0) - bloque.v0*(bloque.tf - bloque.t0) - a[3];
    a[1] = bloque.v0*(bloque.tf - bloque.t0);
    a[0] = bloque.p0;

    lambda = (time_current - bloque.t0)/(bloque.tf - bloque.t0);
    float lambda_2 = lambda*lambda;
    float lambda_3 = lambda * lambda_2;

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

void convert_speed(struct SpeedConverterBlock &bloque, struct RobotParamater &param){

    bloque.v = (bloque.v_rigth + bloque.v_left)/2;
    bloque.w = (bloque.v_rigth - bloque.v_left)/(2*param.R);

}

float convert_degrees2radius(float degrees){
   float radius;
    return radius = (degrees*2*M_PI)*360; 
}

int main(int argv, char *argc []){

    // Blocks definition
    struct GenerationTrajectoireBlock block_trajectory = {.p0 = 0, .pf = 10, .v0 = 0, .vf = 0, .t0 = 0, .tf = 5, .q = 0, .dq = 0};
    struct ReferenceChangeBlock reference_change_block = {.x = 0, .y = 0, .theta = 0, .x_head = 0, .y_head = 0, .theta_head = 0 };
    struct SpeedConverterBlock speed_converter_block = {.v_rigth = 0, .v_left  = 0, .v = 0, .w = 0};
    struct GainSpeedBlock gain_speed_block = {.v_desired = 0, .w_desired = 0, .v_turtle = 0, .w_turtle = 0};
    struct M_inverseBlock inverse_matrix_block = {.ux = 0, .uy = 0, .theta = 0, .v_desired = 0, .w_desired = 0};
    struct IntegratorBlock itegrator_block = {.dx = 0, .dy = 0, .dtheta = 0, .initial_x = 0, .initial_y = 0, .initial_theta = 0, .x = 0, .y = 0, .theta = 0 };
    

    // Robot's structs
    struct RobotParamater robot_paramaters = {.Km = 1, .tau = 0.025, .R = 0.17};
    struct TurtlebotBlock turtlebot2 = {.v_turtle = 0, .w_turtle = 0, .theta = 0, .dx = 0, .dy = 0, .dtheta = 0};
  
    float t = 0;
    float t_current;
    float delta = 0.01;

    for(t = 0; t < 10; t = t + delta){

        inverse_matrix_block.ux = 2;
        inverse_matrix_block.uy = 2;
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

        calcule_integrator(itegrator_block, delta);
        
        //std::cout << itegrator_block.x << " , " << itegrator_block.y << " , " << itegrator_block.theta << '\n';

        reference_change_block.x = itegrator_block.x;
        reference_change_block.y = itegrator_block.y;
        reference_change_block.theta = itegrator_block.theta;

        update_CoordsHead(reference_change_block,robot_paramaters);

        std::cout << reference_change_block.x_head << " , " << reference_change_block.y_head << " , " << reference_change_block.theta_head << '\n';


    }

    return 0;
}