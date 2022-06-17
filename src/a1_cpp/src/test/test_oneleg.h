// Test one leg motion.
// Created by ang on 06.16
//
// 1. Given footholds on the ground and generate intermidiate swing foot traj by Bezier.
// 2. Read joint config q from motor angles.
// 3. Compare current foot pos (fk q) with desired pos, use this error to gen torques, then send torques to motors.
#ifndef A1_CPP_TEST_ONELEG_H
#define A1_CPP_TEST_ONELEG_H

#include <cstdio>
#include <vector>

#include <Eigen/Dense>
#include "../utils/Utils.h"
#include "../A1CtrlStates.h"  // state
#include "../A1Params.h"
#include "../legKinematics/A1Kinematics.h"

class TestOneLeg {
public:
    TestOneLeg();

    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;  // Eigen::Vector3d q 
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;

    // void update_motor_foot(A1CtrlStates &state){ }

    BezierUtils bezierUtils[NUM_LEG];

private:
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};



};


#endif 