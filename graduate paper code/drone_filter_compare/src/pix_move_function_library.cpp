#ifndef __PIX_MOVE_FUC__
#define __PIX_MOVE_FUC__

#include <pix_library_ass.h>

#include "pix_control.h"

void PXDroneControl::ZeroVelocity(void) {
    Body_Vel_Input[_X] = 0.0;
    Body_Vel_Input[_Y] = 0.0;
    Body_Vel_Input[_Z] = 0.0;
}

bool PXDroneControl::MoveToGoalVel(void) {
    linear_speed_pid.PID_set(pid_dt, pid_max, pid_min, pid_Kp, pid_Kd, pid_Ki);
    z_linear_speed_pid.PID_set(pid_th_dt, pid_z_max, pid_z_min, pid_z_Kp, pid_z_Kd, pid_z_Ki);
    th_w_pid.PID_set(pid_th_dt, pid_th_max, pid_th_min, pid_th_Kp, pid_th_Kd, pid_th_Ki);
    uint16_t seq = 0;
    current_move_check_flag = true;
    ros::Rate move_rate(50);
    double_t tmp_goal_x = Pre_Goal_Pose_[_X];
    double_t tmp_goal_y = Pre_Goal_Pose_[_Y];
    double_t tmp_goal_z = Pre_Goal_Pose_[_Z];
    double_t tmp_goal_yaw = Pre_Yaw_Goal_;
    Pre_Goal_Pose_[_X] = Goal_pose[_X];
    Pre_Goal_Pose_[_Y] = Goal_pose[_Y];
    Pre_Goal_Pose_[_Z] = Goal_pose[_Z];
    Other_Task_Flag = true;
    double_t diff_x = tmp_goal_x - Pre_Goal_Pose_[_X];
    double_t diff_y = tmp_goal_y - Pre_Goal_Pose_[_Y];
    Yaw_Goal = atan2(diff_y, diff_x);
    ROS_INFO("diff : %f %f", diff_x, diff_y);
    ROS_INFO("1. %f ,%f", RAD_TO_DEG(Yaw_Goal), RAD_TO_DEG(Body_RPY_Cur[_YAW]));
    while (ros::ok()) {
        if (stop_flag || Lidar_Emergency_Flag) {
            Vel_msg.linear.x = 0;
            Vel_msg.linear.y = 0;
            Vel_msg.linear.z = 0;
            Vel_msg.angular.z = 0;
            Goal_pose[_X] = Estimated_Pose[_X];
            Goal_pose[_Y] = Estimated_Pose[_Y];
            Goal_pose[_Z] = Estimated_Pose[_Z];
            Yaw_Goal = Body_RPY_Cur[_YAW];
            current_move_check_flag = false;
            stop_flag = false;
            Lidar_Emergency_Flag = false;
            Other_Task_Flag = false;
            return false;
        }
        if (navi_mode == "rtr") {
            switch (seq) {
                case _ROTATE_: {
                    if (RotateAlgorithm(Yaw_Goal, Body_RPY_Cur[_YAW], DEG_TO_RAD(5.0))) {
                        Goal_pose[_X] = tmp_goal_x;
                        Goal_pose[_Y] = tmp_goal_y;
                        Goal_pose[_Z] = tmp_goal_z;
                        ROS_INFO("1. %f", RAD_TO_DEG(Body_RPY_Cur[_YAW]));
                        seq++;
                    }
                    break;
                }

                case _LINEAR_MOVE_: {
                    double distance = Linear_Flight_Algorithm(Yaw_Goal);
                    // ROS_INFO("2. %f", distance);
                    if (distance < Threshold_Distance) {
                        seq++;
                        Yaw_Goal = tmp_goal_yaw;
                        ROS_INFO("3. %f ,%f", RAD_TO_DEG(Yaw_Goal), RAD_TO_DEG(Body_RPY_Cur[_YAW]));
                    }
                    break;
                }
                case _FINAL_ROTATE_: {
                    if (RotateAlgorithm(Yaw_Goal, Body_RPY_Cur[_YAW])) {
                        current_move_check_flag = false;
                        ROS_INFO("finish");
                        return true;
                    }
                    break;
                }
            }
        } else {
            Goal_pose[_X] = tmp_goal_x;
            Goal_pose[_Y] = tmp_goal_y;
            Goal_pose[_Z] = tmp_goal_z;
            Yaw_Goal = tmp_goal_yaw;
            if (Linear_Flight_Algorithm(Yaw_Goal) < Threshold_Distance_free_mode || Arrive_Flag) {
                if (Satisfy_Yaw_Flag) {
                    ROS_INFO_STREAM("Satisfy the Condition");
                    Satisfy_Yaw_Flag = false;
                    Arrive_Flag = false;
                    current_move_check_flag = false;
                    Other_Task_Flag = false;
                    return true;
                }
                Arrive_Flag = true;
                ZeroVelocity();
                ROS_INFO_STREAM("Only Rotation");
            }
        }
        move_rate.sleep();
        ros::spinOnce();
    }
    return false;
}

bool PXDroneControl::Take_Off(double_t altitude, double_t takeoff_vel) {
    set_mode.request.custom_mode = "OFFBOARD";
    if (!drone_setmode_client.call(set_mode)) {
        ROS_ERROR_STREAM("Set mode Error");
        return false;
    } else {
        ROS_INFO_STREAM("set mode complete");
    }
    arm_cmd.request.value = true;
    if (!drone_arm_client.call(arm_cmd)) {
        ROS_ERROR_STREAM("Drone armming Error");
        return false;
    } else {
        ROS_INFO_STREAM("arming complete");
    }
    Goal_pose[_Z] = altitude;
}

bool PXDroneControl::Landing(void) {
    set_mode.request.custom_mode = "AUTO.LAND";
    if (!drone_setmode_client.call(set_mode)) {
        return false;
    }
    takeoff_land_msgs.request.yaw = 0.0;
    takeoff_land_msgs.request.latitude = 0.0;
    takeoff_land_msgs.request.longitude = 0.0;
    takeoff_land_msgs.request.altitude = 0.0;
    if (!drone_land_client.call(takeoff_land_msgs)) {
        return false;
    }
    return true;
}

void PXDroneControl::ReturnToHome(double_t altitude) {
    std::vector<double_t> rth_param;
    rth_param.emplace_back(Goal_pose[_X]);
    rth_param.emplace_back(Goal_pose[_Y]);
    rth_param.emplace_back(altitude);
    rth_param.emplace_back(Yaw_Goal);

    rth_param.emplace_back(0.0);
    rth_param.emplace_back(0.0);
    rth_param.emplace_back(altitude);
    rth_param.emplace_back(0.0);

    waypoint_vector.clear();
    Waypoint temp;
    for (uint8_t index = 0; index < 8; index++) {
        if (index % 4 == 0) {
            temp.x_goal = rth_param[index];
        } else if (index % 4 == 1) {
            temp.y_goal = rth_param[index];
        } else if (index % 4 == 2) {
            temp.z_goal = rth_param[index];
        } else if (index % 4 == 3) {
            temp.yaw_goal = rth_param[index];
            waypoint_vector.emplace_back(temp);
        }
    }
    MoveWaypoint();
    if (!Landing()) ROS_ERROR_STREAM("Landing ERROR");
}

void PXDroneControl::MoveWaypoint(void) {
    for (uint8_t i = 0; i < waypoint_vector.size(); i++) {
        Pre_Goal_Pose_[_X] = waypoint_vector[i].x_goal;
        Pre_Goal_Pose_[_Y] = waypoint_vector[i].y_goal;
        Pre_Goal_Pose_[_Z] = waypoint_vector[i].z_goal;
        Pre_Yaw_Goal_ = waypoint_vector[i].yaw_goal;

        ROS_INFO("%lf %lf %lf %lf", waypoint_vector[i].x_goal, waypoint_vector[i].y_goal, waypoint_vector[i].z_goal, waypoint_vector[i].yaw_goal);

        if (!MoveToGoalVel()) {
            return;
        }
    }
}

void PXDroneControl::ManualControl(void) {
    char key(' ');
    ros::Rate mcotrol_rate(10);
    while (ros::ok()) {
        key = ' ';
        key = getch();
        Other_Task_Flag = true;
        switch (key) {
            case '\x03':
                Other_Task_Flag = false;
                Goal_pose[_X] = Estimated_Pose[_X];
                Goal_pose[_Y] = Estimated_Pose[_Y];
                Goal_pose[_Z] = Estimated_Pose[_Z];
                Yaw_Goal = Body_RPY_Cur[_YAW];
                return;
            case 'p':
                Other_Task_Flag = false;
                Goal_pose[_X] = Estimated_Pose[_X];
                Goal_pose[_Y] = Estimated_Pose[_Y];
                Goal_pose[_Z] = Estimated_Pose[_Z];
                Yaw_Goal = Body_RPY_Cur[_YAW];
                return;
            case 'r':
                Body_Vel_Input[_X] = 0;
                Body_Vel_Input[_Y] = 0;
                Body_Vel_Input[_Z] += 0.1;
                Yaw_Rate_Input = 0;
                break;
            case 'f':
                Body_Vel_Input[_X] = 0;
                Body_Vel_Input[_Y] = 0;
                Body_Vel_Input[_Z] -= 0.1;
                Yaw_Rate_Input = 0;
                break;
            case 'w':
                Body_Vel_Input[_X] = 0;
                Body_Vel_Input[_Y] += 0.1;
                Body_Vel_Input[_Z] = 0;
                Yaw_Rate_Input = 0;
                break;

            case 's':
                Body_Vel_Input[_X] = 0;
                Body_Vel_Input[_Y] -= 0.1;
                Body_Vel_Input[_Z] = 0;
                Yaw_Rate_Input = 0;
                break;

            case 'a':
                Body_Vel_Input[_X] -= 0.1;
                Body_Vel_Input[_Y] = 0;
                Body_Vel_Input[_Z] = 0;
                Yaw_Rate_Input = 0;
                break;

            case 'd':
                Body_Vel_Input[_X] += 0.1;
                Body_Vel_Input[_Y] = 0;
                Body_Vel_Input[_Z] = 0;
                Yaw_Rate_Input = 0;
                break;
            case 'q':
                Body_Vel_Input[_X] = 0;
                Body_Vel_Input[_Y] = 0;
                Body_Vel_Input[_Z] = 0;
                Yaw_Rate_Input += 0.1;
                break;
            case 'e':
                Body_Vel_Input[_X] = 0;
                Body_Vel_Input[_Y] = 0;
                Body_Vel_Input[_Z] = 0;
                Yaw_Rate_Input -= -0.1;
                break;
            default:
                Body_Vel_Input[_X] = 0;
                Body_Vel_Input[_Y] = 0;
                Body_Vel_Input[_Z] = 0;
                Yaw_Rate_Input = 0;
                break;
        }
        mcotrol_rate.sleep();
        ros::spinOnce();
    }
}

#endif
