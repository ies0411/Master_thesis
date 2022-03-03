#ifndef __PIX_LINEAR_FLY_LIB__
#define __PIX_LINEAR_FLY_LIB__

#include <pix_control.h>
#include <pix_library_ass.h>

/**
 * @brief linear flight algorithm function
 * @details go straight to goal position
 * getting a GPS & IMU data and transfer global pose to local pose
 * using PD controller
 * @author Ethan.lim
 * @date '21.01.26
 */

/**calculation distance between two points**/
void PXDroneControl::Two_Point_Distance(const std::vector<std::pair<double, double>> &point, double_t &distance) {
    distance = sqrt(pow(point[0].first - point[0].second, 2) + pow(point[1].first - point[1].second, 2));
}

/**calculation between current position and goal position**/
void PXDroneControl::Distance(const std::vector<std::pair<double, double>> &point, double_t &distance) {
    distance = sqrt(pow(point[0].first - point[0].second, 2) + pow(point[1].first - point[1].second, 2) + pow(point[2].first - point[2].second, 2));
}

void PXDroneControl::TransRadTOPitoPi(double_t &before_rad, double_t &after_rad) {
    if (before_rad > M_PI) {
        after_rad = before_rad - (M_PI * 2.0);
    } else if (before_rad < (-1) * M_PI) {
        after_rad = before_rad + (M_PI * 2.0);
    } else {
        after_rad = before_rad;
    }
}

bool PXDroneControl::RotateAlgorithm(double_t &goal_yaw, double_t &yaw, double_t threshold) {
    TransRadTOPitoPi(goal_yaw, goal_yaw);
    TransRadTOPitoPi(yaw, yaw);

    double_t diff_yaw = goal_yaw - yaw;
    TransRadTOPitoPi(diff_yaw, diff_yaw);

    double_t fabs_th = fabs(diff_yaw);
    double_t turn_pid;
    turn_pid = th_w_pid.calculate(0.0, fabs_th);

    Turn_Speed = turn_pid;
    if (fabs_th < threshold) {
        Yaw_Rate_Input = 0;
        return true;
    } else {
        if (diff_yaw < 0) {
            Yaw_Rate_Input = (-1) * Turn_Speed;
            return false;
        } else {
            Yaw_Rate_Input = Turn_Speed;
            return false;
        }
    }
}

double PXDroneControl::Linear_Flight_Algorithm(double_t &goal_yaw, double_t threshold) {
    Eigen::MatrixXf Tr(2, 2), pre_pose(2, 1), Tr_pose(2, 1), pre_goal_pose(2, 1), Tr_goal_pose(2, 1);
    double_t th = -Body_RPY_Cur[_YAW];

    Tr << cos(th), -sin(th),
        sin(th), cos(th);
    pre_pose << Estimated_Pose[_X], Estimated_Pose[_Y];
    Tr_pose = Tr * pre_pose;

    double_t transfer_pose_x = Tr_pose(0);
    double_t transfer_pose_y = Tr_pose(1);

    pre_goal_pose << Goal_pose[_X], Goal_pose[_Y];
    Tr_goal_pose = Tr * pre_goal_pose;

    double_t transfer_pose_x_goal = Tr_goal_pose(0);
    double_t transfer_pose_y_goal = Tr_goal_pose(1);
    std::vector<std::pair<double_t, double_t>> point;

    point.emplace_back(std::make_pair(transfer_pose_x, transfer_pose_x_goal));
    point.emplace_back(std::make_pair(transfer_pose_y, transfer_pose_y_goal));
    point.emplace_back(std::make_pair(Estimated_Pose[_Z], Goal_pose[_Z]));

    double_t distance;
    Distance(point, distance);
    double_t x_y_distance;
    Two_Point_Distance(point, x_y_distance);
    // ROS_INFO("dis : %f %f",)

    double_t diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
    double_t diff_pose_y = transfer_pose_y_goal - transfer_pose_y;

    double_t diff_pose_z = Goal_pose[_Z] - Estimated_Pose[_Z];

    double_t tan_degree = atan2(diff_pose_y, diff_pose_x);

    double_t speed_pid = linear_speed_pid.calculate(0, x_y_distance);
    double_t z_speed_pid = z_linear_speed_pid.calculate(0, fabs(diff_pose_z));

    Plane_Speed = speed_pid;

    Body_Vel_Input[_Y] = (Plane_Speed)*sin(tan_degree);
    Body_Vel_Input[_X] = (Plane_Speed)*cos(tan_degree);

    Z_Speed = z_speed_pid;

    if (diff_pose_z > 0) {
        Body_Vel_Input[_Z] = Z_Speed;
    } else {
        Body_Vel_Input[_Z] = (-1) * Z_Speed;
    }
    Satisfy_Yaw_Flag = RotateAlgorithm(goal_yaw, Body_RPY_Cur[_YAW], threshold);

    return distance;
}

#endif
