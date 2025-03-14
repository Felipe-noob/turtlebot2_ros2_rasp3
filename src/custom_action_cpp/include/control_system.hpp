#ifndef _CONTROL_SYS_H
#define _CONTROL_SYS_H

// ========================== Structs for the real robot

struct RobotParamater{
    double Km;       // Static Gain [m*s*V]
    double tau;      // Time constant [s]
    double R;        // Turtlebot base radius [m]

    double V_max;
    double W_max;
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

// ========================== Structs the system control 

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

struct M_inverseBlock{

    // input
    double ux;
    double uy;
    double theta;

    //output
    double v_desired;
    double w_desired;

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

// ========================== Structs for turtle dynamic simulation

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


double convert_degrees2radians(double degrees);
void update_CoordsHead(struct ReferenceChangeBlock &reference_change_block, struct IntegratorBlock itegrator_block,struct RobotParamater &param);
void update_TrajectoryOutput(struct GenerationTrajectoireBlock &bloque, double time_current);
void update_GainSpeedBlock(struct GainSpeedBlock &gain_speed_block, struct M_inverseBlock &inverse_matrix_block);
void update_M_inverseBlock(struct M_inverseBlock &inverse_matrix_block,struct CommandBlock &command_block, struct RobotParamater &param, double theta);
void update_TurtlebotBlock(struct TurtlebotBlock &turtlebot2, struct GainSpeedBlock &gain_speed_block, double theta);
void update_integrator(struct IntegratorBlock &itegrator_block, struct TurtlebotBlock &turtlebot2 ,double dt);
void update_SaturationBlock(struct SaturationBlock &bloque, struct RobotParamater &param);
void update_CommandBlock(struct CommandBlock &command_block,struct GenerationTrajectoireBlock &block_trajectory_X, struct GenerationTrajectoireBlock &block_trajectory_Y , struct ReferenceChangeBlock reference_change_block, double gain_K);
int sign(double x);


#endif