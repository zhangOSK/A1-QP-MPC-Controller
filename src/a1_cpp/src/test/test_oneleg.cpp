#include "test_oneleg.h"

TestOneLeg::TestOneLeg() {
    std::cout << "init TestOneLeg" << std::endl;
    // the leg kinematics is relative to body frame, which is the center of the robot
    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.20;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = 0.20;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    joint_pos.setZero();
    joint_vel.setZero();
    joint_torques.setZero();

}

int main(int, char**) {
    TestOneLeg oneleg;
    A1CtrlStates state;
    A1Kinematics a1_kin;
    std::cout<< "a1_kin, rho_opt_size = "<<a1_kin.RHO_OPT_SIZE <<std::endl;
    // init values
    state.reset(); // 0, 1, 2, 3: FL, FR, RL, RR
    state.robot_mass = 20; //note 1
    state.a1_trunk_inertia << 0.0158533, 0.0, 0.0,
            0.0, 0.0377999, 0.0,
            0.0, 0.0, 0.0456542;  //note 2
    state.root_euler << 0.0, 0.0, 0.0;

    double dt = 0.01; // TODO: update dt = ros::Time::now - pre
    // BezierUtils bezierUtils[NUM_LEG];

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;    
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    foot_pos_target.setZero();
    foot_vel_target.setZero();
    foot_pos_cur.setZero();
    foot_vel_cur.setZero();
    foot_pos_rel_last_time.setZero();
    foot_pos_target_last_time.setZero();
    foot_pos_error.setZero();
    foot_vel_error.setZero();
    foot_forces_kin.setZero();
    joint_torques.setZero();

    Eigen::Vector3d foot_pos_start(0.2,0.2,-0.33);  // TODO: re-check x,y,z to avoid un-desired collision 
    Eigen::Vector3d foot_pos_final(0.25,0.2,-0.33);

    BezierUtils bs_utils;
    Eigen::MatrixXd interp_pos_rst(3,50);
    int i = 0;
    for (double t=0.0; t<1.0; t += 0.02) {
        interp_pos_rst.col(i) = bs_utils.get_foot_pos_curve(t,foot_pos_start, 
        foot_pos_final,0);
        i++;
    }
    std::cout<<"position interpolation"<<std::endl;
    std::cout<<interp_pos_rst.col(45)<<std::endl;

    for (int t = 0; t < 50; t++){
        // !!! TODO: READ q FROM MOTOR
        for (int i = 0; i < NUM_LEG; ++i) {
            if (i == 0) {  
                // DO forward kinematics to get foot pos
                // after getting q, calculate current foot pos and update jac     
                state.foot_pos_rel.block<3, 1>(0, i) = a1_kin.fk(
                        state.joint_pos.segment<3>(3 * i),
                        oneleg.rho_opt_list[i], oneleg.rho_fix_list[i]);  // foot pos in robot frame  ! undefined reference: add A1Kinematics.cpp in cmakelist. 
                foot_pos_cur.block<3, 1>(0, i) = state.foot_pos_rel.col(i) ;
                state.j_foot.block<3, 3>(3 * i, 3 * i) = a1_kin.jac(
                        state.joint_pos.segment<3>(3 * i),
                        oneleg.rho_opt_list[i], oneleg.rho_fix_list[i]);  // jacobian

                // foot_pos_cur.block<3, 1>(0, i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3, 1>(0, i); // robot frame 
                std::cout<< "t = " << t << std::endl;
                // std::cout<< ", cur, " << foot_pos_cur.col(0) << std::endl;
                
                foot_vel_cur.block<3, 1>(0, i) = (foot_pos_cur.block<3, 1>(0, i) - foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
                foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

                foot_vel_target.block<3, 1>(0, i) = (foot_pos_target.block<3, 1>(0, i) - foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
                foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

                foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
                foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
                foot_forces_kin.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                                                    foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i)); // note 3, kp, kd

                state.foot_pos_cur = foot_pos_cur;
                state.foot_forces_kin = foot_forces_kin; // note 4

                //!!!! UPDATE JACOBIAN state.j_foot

                // compute torques
                Eigen::Matrix3d jac = state.j_foot.block<3, 3>(3 * i, 3 * i);
                Eigen::Vector3d force_tgt = state.km_foot.cwiseProduct(state.foot_forces_kin.block<3, 1>(0, i));
                joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt);   // jac * tau = F
            }
            joint_torques += state.torques_gravity;
            // prevent nan
            for (int i = 0; i < 12; ++i) {
                if (!isnan(joint_torques[i]))
                    state.joint_torques[i] = joint_torques[i];
            }

        }

    }

}
