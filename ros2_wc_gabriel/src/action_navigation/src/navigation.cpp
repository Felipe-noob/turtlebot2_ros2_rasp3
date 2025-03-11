
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

    // Robot's structs
    struct RobotParamater robot_paramaters = {.Km = 1, .tau = 0.025, .R = 0.17};
  
    float i = 0;
    float t_current;

    // Test update_TrajectoryOutput
    std::cout << "Test update_TrajectoryOutput" << '\n';

    for(i = 0 ; i < 5 ; i = i+0.1){

        t_current = i;
        update_TrajectoryOutput(block_trajectory,t_current);
        std::cout << block_trajectory.q << '\n';
    }

    // Test update_CoordsHead

    std::cout << "Test update_CoordsHead" << '\n';

    for(i = 0 ; i < 5 ; i = i+0.1){
        reference_change_block.x = i;
        reference_change_block.y = i;

        if(i > 2) {
            reference_change_block.theta += 0.5 ;
        }

        update_CoordsHead(reference_change_block,robot_paramaters);
        
        std::cout << reference_change_block.x_head << " , " << reference_change_block.y_head << " , " << reference_change_block.theta_head << '\n';
    }

    // Test convert_speed
    
    std::cout << "Test convert_speed" << '\n';

      for(i = 0 ; i < 5 ; i = i+0.1){
        speed_converter_block.v_rigth = 1;
        speed_converter_block.v_left = 1;

        if(i > 2) {
            speed_converter_block.v_rigth = 1;
            speed_converter_block.v_left = 2;
        }

        convert_speed(speed_converter_block,robot_paramaters);
        
        std::cout << speed_converter_block.v << " , " << speed_converter_block.w << '\n';
    }

    return 0;
}