#ifndef __PIX_FILTER__
#define __PIX_FILTER__

#include <pix_library_ass.h>

#include "pix_control.h"

void PXDroneControl::EKF_Filter(void) {
    ros::Time after_filter;
    after_filter = ros::Time::now();
    ros::Duration duration = after_filter - filter_begin;
    double dt = duration.toSec();

    A_EKF << 1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, dt, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, dt,
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    Ideal_Pose[_X] = EKF_Pose[_X] + dt * Body_Vel_Input[_X] * cos(Body_RPY_Cur[_YAW]) - dt * Body_Vel_Input[_Y] * sin(Body_RPY_Cur[_YAW]);
    Ideal_Pose[_Y] = EKF_Pose[_Y] + dt * Body_Vel_Input[_X] * sin(Body_RPY_Cur[_YAW]) + dt * Body_Vel_Input[_Y] * cos(Body_RPY_Cur[_YAW]);
    Ideal_Pose[_Z] = EKF_Pose[_Z] + dt * Body_Vel_Input[_Z];

    Xp_EKF(0, 0) = EKF_Pose[_X] + dt * EKF_Vel[_X] + 0.5 * cos(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_X] * dt * dt - 0.5 * sin(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_Y] * dt * dt;
    Xp_EKF(1, 0) = EKF_Pose[_Y] + dt * EKF_Vel[_Y] + 0.5 * sin(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_X] * dt * dt + 0.5 * cos(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_Y] * dt * dt;
    Xp_EKF(2, 0) = EKF_Pose[_Z] + EKF_Vel[_Z] * dt + Body_Acc_Cur[_Z] * 0.5 * dt * dt;
    Xp_EKF(3, 0) = EKF_Vel[_X] + cos(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_X] * dt - sin(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_Y] * dt;
    Xp_EKF(4, 0) = EKF_Vel[_Y] + sin(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_X] * dt + cos(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_Y] * dt;
    Xp_EKF(5, 0) = EKF_Vel[_Z] + Body_Acc_Cur[_Z] * dt;

    // std::cout<<Xp_EKF<<std::endl;
    Pp_EKF = A_EKF * P_EKF * A_EKF.transpose() + Q_EKF;

    Eigen::MatrixXf temp;

    temp = H_EKF * Pp_EKF * H_EKF.transpose() + R_EKF;
    K_EKF = Pp_EKF * H_EKF.transpose() * temp.inverse();

    X_EKF = Xp_EKF + K_EKF * (Z_EKF - Xp_EKF);

    std::vector<std::pair<double_t, double_t>> ideal_pose_vec;

    ideal_pose_vec.emplace_back(std::make_pair(Ideal_Pose[_X], Xp_EKF(0, 0)));
    ideal_pose_vec.emplace_back(std::make_pair(Ideal_Pose[_Y], Xp_EKF(1, 0)));
    ideal_pose_vec.emplace_back(std::make_pair(Ideal_Pose[_Z], Xp_EKF(2, 0)));

    double_t est_distance, measured_distance;
    // if(idea)
    Distance(ideal_pose_vec, est_distance);
    ideal_pose_vec.clear();

    ideal_pose_vec.emplace_back(std::make_pair(Ideal_Pose[_X], Z_EKF(0, 0)));
    ideal_pose_vec.emplace_back(std::make_pair(Ideal_Pose[_Y], Z_EKF(1, 0)));
    ideal_pose_vec.emplace_back(std::make_pair(Ideal_Pose[_Z], Z_EKF(2, 0)));

    Distance(ideal_pose_vec, measured_distance);

    double_t alpha = ((est_distance) / (est_distance + measured_distance));
    // ROS_INFO("%f ", alpha);
    // ROS_INFO("!! %f", ((measured_distance) / (est_distance + measured_distance)));
    Q_EKF = alpha * Q_EKF_Init;  // 0.12 * Eigen::MatrixXf::Identity(6, 6);
    R_EKF = (1 - alpha) * R_EKF_Init;
    // (est_distance) / (est_distance + measured_distance);

    // ROS_INFO("%f %f", est_distance, mesured_distance);

    EKF_Pose[_X] = X_EKF(0);
    EKF_Pose[_Y] = X_EKF(1);
    EKF_Pose[_Z] = X_EKF(2);

    EKF_Vel[_X] = X_EKF(3);
    EKF_Vel[_Y] = X_EKF(4);
    EKF_Vel[_Z] = X_EKF(5);

    P_EKF = Pp_EKF - K_EKF * H_EKF * Pp_EKF;
    filter_begin = ros::Time::now();

    if (pose_estimation_check_) {
        ROS_INFO("ekf pose : %f, %f , %f", EKF_Pose[_X], EKF_Pose[_Y], EKF_Pose[_Z]);
    }
}

// void PXDroneControl::SigmaPoint(Eigen::MatrixXf &X, Eigen::MatrixXf &P, float kappa) {
//     Xi = Eigen::MatrixXf::Zero(X_n, UKF_Number * X_n + 1);
//     W = Eigen::MatrixXf::Zero(UKF_Number * X_n + 1, 1);
//     printf("1\n");

//     Xi.col(0) = X_UKF;
//     printf("2\n");

//     W(0) = kappa / (X_n + kappa);

//     Eigen::MatrixXf Temp;
//     Temp = (X_n + kappa) * P_UKF;
//     Eigen::MatrixXf L(Temp.llt().matrixL());
//     printf("3\n");

//     for (int i = 0; i < (X_n * UKF_Number) / 2; i++) {
//         Xi.col(i + 1) = X + L.row(i).transpose();
//         W(i + 1) = 1 / (2 * (X_n + kappa));
//         Xi.col(X_n + i + 1) = X - L.row(i).transpose();
//         W(X_n + i + 1) = 1 / (2 * (X_n + kappa));
//     }
// }
// void PXDroneControl::SigmaPoint(Eigen::MatrixXf &X, Eigen::MatrixXf &P, float kappa) {
//     Xi = Eigen::MatrixXf::Zero(X_n, UKF_Number * 2 * X_n + 1);
//     W = Eigen::MatrixXf::Zero(UKF_Number * 2 * X_n + 1, 1);

//     Eigen::MatrixXf Temp;
//     Temp = (X_n + kappa) * P_UKF;
//     Eigen::MatrixXf L(Temp.llt().matrixL());
//     Xi.col(0) = X_UKF;
//     W(0) = kappa / (X_n + kappa);
//     for (int j = 0; j < UKF_Number; j++) {
//         Eigen::MatrixXf L_tmp = pow(0.1, j) * L;
//         // / (j + 1.0);

//         // std::cout << L << std::endl;
//         // printf("2\n");
//         if (j == 0) {
//             for (int i = 0; i < X_n; i++) {
//                 Xi.col(i + 1) = X + L.row(i).transpose();

//                 W(i + 1) = 1 / (2 * (X_n + kappa));

//                 Xi.col(X_n + i + 1) = X - L.row(i).transpose();
//                 W(X_n + i + 1) = 1 / (2 * (X_n + kappa));
//             }
//         } else {
//             for (int i = j * X_n; i < j * X_n + X_n; i++) {
//                 Xi.col(i + 1) = X + L_tmp.row(i - j * X_n).transpose();

//                 W(i + 1) = 1 / (2 * (X_n + kappa));

//                 Xi.col(X_n + i + 1) = X - L_tmp.row(i - j * X_n).transpose();
//                 W(X_n + i + 1) = 1 / (2 * (X_n + kappa));
//             }
//         }
//     }
//     // std::cout << Xi << std::endl;
// }

void PXDroneControl::SigmaPoint(Eigen::MatrixXf &X, Eigen::MatrixXf &P, float kappa) {
    Xi = Eigen::MatrixXf::Zero(X_n, 2 * X_n + 1);
    W = Eigen::MatrixXf::Zero(2 * X_n + 1, 1);

    Xi.col(0) = X_UKF;

    W(0) = kappa / (X_n + kappa);

    Eigen::MatrixXf Temp;
    Temp = (X_n + kappa) * P_UKF;
    Eigen::MatrixXf L(Temp.llt().matrixL());

    for (int i = 0; i < X_n; i++) {
        Xi.col(i + 1) = X + L.row(i).transpose();
        W(i + 1) = 1 / (2 * (X_n + kappa));
        Xi.col(X_n + i + 1) = X - L.row(i).transpose();
        W(X_n + i + 1) = 1 / (2 * (X_n + kappa));
    }
}

void PXDroneControl::UT(Eigen::MatrixXf &Xi, Eigen::MatrixXf &W, Eigen::MatrixXf &noiseCov, uint8_t type) {
    uint8_t n = Xi.rows();
    uint8_t kmax = Xi.cols();

    Eigen::MatrixXf Temp_X;
    Temp_X = Eigen::MatrixXf::Zero(6, 1);

    for (int i = 0; i < kmax; i++) {
        Temp_X = Temp_X + W(i) * Xi.col(i);
    }
    Eigen::MatrixXf xcov;
    xcov = Eigen::MatrixXf::Zero(n, n);
    for (int i = 0; i < kmax; i++) {
        xcov = xcov + W(i) * (Xi.col(i) - Temp_X) * (Xi.col(i) - Temp_X).transpose();
    }
    xcov = xcov + noiseCov;
    // std::cout << xcov << std::endl;
    if (type == _UKF_X) {
        Xp_UKF = Temp_X;
        Pp_UKF = Eigen::MatrixXf::Zero(n, n);
        Pp_UKF = xcov;
    } else if (type == _UKF_Z) {
        Zp_UKF = Temp_X;
        Pz_UKF = Eigen::MatrixXf::Zero(n, n);
        Pz_UKF = xcov;
    }
}

Eigen::MatrixXf PXDroneControl::CalFunction(Eigen::MatrixXf xhat) {
    Eigen::MatrixXf Xp(6, 1);
    //   Xp=Eigen::MatrixXf::Zero(6,1);
    ros::Time after_filter;
    after_filter = ros::Time::now();
    ros::Duration duration = after_filter - filter_begin;
    double dt = duration.toSec();
    Xp(0, 0) = xhat(0, 0) + dt * xhat(3, 0) + 0.5 * cos(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_X] * dt * dt - 0.5 * sin(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_Y] * dt * dt;
    Xp(1, 0) = xhat(1, 0) + dt * xhat(4, 0) + 0.5 * sin(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_X] * dt * dt + 0.5 * cos(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_Y] * dt * dt;
    Xp(2, 0) = xhat(2, 0) + xhat(5, 0) * dt + Body_Acc_Cur[_Z] * 0.5 * dt * dt;
    Xp(3, 0) = xhat(3, 0) + cos(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_X] * dt - sin(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_Y] * dt;
    Xp(4, 0) = xhat(4, 0) + sin(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_X] * dt + cos(Body_RPY_Cur[_YAW]) * Body_Acc_Cur[_Y] * dt;
    Xp(5, 0) = xhat(5, 0) + Body_Acc_Cur[_Z] * dt;
    // if (tmp_cnt++ < 10) std::cout << Xp << std::endl;
    filter_begin = ros::Time::now();
    return Xp;
}

void PXDroneControl::UKF_Filter(void) {
    SigmaPoint(X_UKF, P_UKF, 2);

    Eigen::MatrixXf FXi;
    FXi = Eigen::MatrixXf::Zero(X_n, 2 * X_n + 1);

    for (int i = 0; i < 2 * X_n + 1; i++) {
        FXi.col(i) = CalFunction(Xi.col(i));
    }

    UT(FXi, W, Q_UKF, _UKF_X);

    Eigen::MatrixXf HXi;
    HXi = Eigen::MatrixXf::Zero(X_m, 2 * X_n + 1);
    HXi = FXi;
    // HXi=Z_EKF;
    UT(HXi, W, R_UKF, _UKF_Z);

    Eigen::MatrixXf Pxz;
    Pxz = Eigen::MatrixXf::Zero(X_n, X_m);

    for (int i = 0; i < 2 * X_n + 1; i++) {
        Pxz = Pxz + W(i) * (Xi.col(i) - Xp_UKF) * (HXi.col(i) - Zp_UKF).transpose();
    }

    K_UKF = Pxz * Pz_UKF.inverse();
    X_UKF = Xp_UKF + K_UKF * (Z_UKF - Zp_UKF);
    P_UKF = Pp_UKF - K_UKF * Pz_UKF * K_UKF.transpose();
    Estimated_Pose[_X] = X_UKF(0);
    Estimated_Pose[_Y] = X_UKF(1);
    Estimated_Pose[_Z] = X_UKF(2);
    UKF_Pose[_X] = Estimated_Pose[_X];
    UKF_Pose[_Y] = Estimated_Pose[_Y];
    UKF_Pose[_Z] = Estimated_Pose[_Z];
}

// void PXDroneControl::UKF_Filter(void) {
//     SigmaPoint(X_UKF, P_UKF, 10);

//     Eigen::MatrixXf FXi;
//     FXi = Eigen::MatrixXf::Zero(X_n, UKF_Number * 2 * X_n + 1);
//     for (int i = 0; i < UKF_Number * 2 * X_n + 1; i++) {
//         FXi.col(i) = CalFunction(Xi.col(i));
//     }
//     UT(FXi, W, Q_UKF, _UKF_X);
//     Eigen::MatrixXf HXi;
//     HXi = Eigen::MatrixXf::Zero(X_m, UKF_Number * 2 * X_n + 1);
//     HXi = FXi;
//     // HXi=Z_EKF;
//     UT(HXi, W, R_UKF, _UKF_Z);

//     Eigen::MatrixXf Pxz;
//     Pxz = Eigen::MatrixXf::Zero(X_n, X_m);

//     for (int i = 0; i < UKF_Number * 2 * X_n + 1; i++) {
//         Pxz = Pxz + W(i) * (Xi.col(i) - Xp_UKF) * (HXi.col(i) - Zp_UKF).transpose();
//     }

//     K_UKF = Pxz * Pz_UKF.inverse();
//     X_UKF = Xp_UKF + K_UKF * (Z_UKF - Zp_UKF);
//     P_UKF = Pp_UKF - K_UKF * Pz_UKF * K_UKF.transpose();

//     Estimated_Pose[_X] = X_UKF(0);
//     Estimated_Pose[_Y] = X_UKF(1);
//     Estimated_Pose[_Z] = X_UKF(2);
//     // ROS_INFO("%f , %f , %f", Estimated_Pose[_X], Estimated_Pose[_Y], Estimated_Pose[_Z]);
//     UKF_Pose[_X] = Estimated_Pose[_X];
//     UKF_Pose[_Y] = Estimated_Pose[_Y];
//     UKF_Pose[_Z] = Estimated_Pose[_Z];
// }

#endif
