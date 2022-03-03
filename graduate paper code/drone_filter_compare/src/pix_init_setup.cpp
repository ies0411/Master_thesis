#ifndef __PIX_INIT__
#define __PIX_INIT__
#include <pix_library_ass.h>

#include "pix_control.h"

// void PXDroneControl::InitFlag(void) {
// }

bool PXDroneControl::Init_Set(void) {
    EKF_Vel = Eigen::Vector3d::Zero();
    Pre_Goal_Pose_ = Eigen::Vector3d::Zero();
    Body_Vel_Input = Eigen::Vector3d::Zero();
    Estimated_Pose = Eigen::Vector3d::Zero();
    Estimated_Vel = Eigen::Vector3d::Zero();
    Body_RPY_Cur = Eigen::Vector3d::Zero();
    Body_Acc_Cur = Eigen::Vector3d::Zero();
    Goal_pose = Eigen::Vector3d::Zero();
    Lidar_Scan_Dis = Eigen::Vector3d::Zero();
    GPS_Pose = Eigen::Vector3d::Zero();
    Body_Vel_Measure_Bias = Eigen::Vector3d::Zero();
    EKF_Pose = Eigen::Vector3d::Zero();
    UKF_Pose = Eigen::Vector3d::Zero();
    // GPS_Pose = Eigen::Vector3d::Zero();
    Lidar_Scan_Zero = Eigen::Vector3d::Zero();

    A_EKF = Eigen::MatrixXf::Zero(6, 6);
    Xp_EKF = Eigen::MatrixXf::Zero(6, 1);
    P_EKF = 3.0 * Eigen::MatrixXf::Identity(6, 6);
    Q_EKF = 0.12 * Eigen::MatrixXf::Identity(6, 6);
    R_EKF = 25.0 * Eigen::MatrixXf::Identity(6, 6);
    H_EKF = Eigen::MatrixXf::Identity(6, 6);
    X_EKF = Eigen::MatrixXf::Zero(6, 1);
    Z_EKF = Eigen::MatrixXf::Zero(6, 1);

    P_UKF = 3.0 * Eigen::MatrixXf::Identity(6, 6);
    Q_UKF = 0.12f * Eigen::MatrixXf::Identity(6, 6);  // Q(6,6)=0.2;
    R_UKF = 25.0f * Eigen::MatrixXf::Identity(6, 6);
    H_UKF = Eigen::MatrixXf::Identity(6, 6);
    X_UKF = Eigen::MatrixXf::Zero(6, 1);
    Z_UKF = Eigen::MatrixXf::Zero(6, 1);

    X_n = X_UKF.rows();
    X_m = Z_UKF.rows();

    Xi = Eigen::MatrixXf::Zero(X_n, 2 * X_n + 1);
    W = Eigen::MatrixXf::Zero(2 * X_n + 1, 1);

    /****todo pending while flag set******/
    //  while (ros::ok()) {
    //      ros::spinOnce();
    //      if((Odom_Start_flag_cnt < 50) && (Imu_Start_flag_cnt < 100){
    //          break;
    //      }
    //  }

    linear_speed_pid.PID_set(pid_dt, pid_max, pid_min, pid_Kp, pid_Kd, pid_Ki);
    z_linear_speed_pid.PID_set(pid_th_dt, pid_z_max, pid_z_min, pid_z_Kp, pid_z_Kd, pid_z_Ki);
    th_w_pid.PID_set(pid_th_dt, pid_th_max, pid_th_min, pid_th_Kp, pid_th_Kd, pid_th_Ki);
    filter_begin = ros::Time::now();
    ROS_INFO_STREAM("init finish");
}

#endif
