

#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

struct Generation_Trajectoire{
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

class Trajectory_Block{
    public:

        float t;
        float t0;
        float tf;
        float v0;
        float vf;
        float acc0;
        float accf;
        
        void Generation_Trajectoire_update(struct Generation_Trajectoire &bloque, float time_current){

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

};


int main(int argv, char *argc []){

    Trajectory_Block block1;

    Generation_Trajectoire struct1;

    struct1.p0 = 0;
    struct1.pf = 10;
    struct1.v0 = 0;
    struct1.vf = 0;
    struct1.t0 = 0;
    struct1.tf = 10;
    struct1.q = 0;
    struct1.dq = 0;

   // float t_current = 1;

    float i = 0;
    float t_current;

    for(i = 0 ; i < 5 ; i = i+0.1){

        t_current = i;
        block1.Generation_Trajectoire_update(struct1,t_current);
        std::cout << struct1.q << '\n';
    }

/*
    block1.Generation_Trajectoire_update(struct1,t_current);
        
    std::cout << struct1.q << '\n';
*/
   // block1.Generation_Trajectoire_update();

    return 0;
}