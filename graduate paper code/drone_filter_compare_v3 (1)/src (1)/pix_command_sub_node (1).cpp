#ifndef __PIX_VM_COMMAND_SUB__
#define __PIX_VM_COMMAND_SUB__

#include <pix_library_ass.h>

#include "pix_control.h"

void PXDroneControl::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int PXDroneControl::getch(void) {
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

void PXDroneControl::CommandCallback(const std_msgs::String::ConstPtr& msg) {
    std::istringstream iss(msg->data);
    std::string token;
    std::vector<std::string> param;
    while (getline(iss, token, ' ')) {
        param.emplace_back(token);
    }
    if (param[0] == "takeoff") {
        uint8_t param_size = param.size();
        if (param_size == 1) {
            if (!Take_Off()) ROS_ERROR_STREAM("Take-off ERROR");
        } else if (param_size == 2) {
            if (!Take_Off(stod(param[1]))) ROS_ERROR_STREAM("Take-off ERROR");
        } else if (param_size == 3) {
            if (!Take_Off(stod(param[1]), stod(param[2]))) ROS_ERROR_STREAM("Take-off ERROR");
        } else {
            ROS_ERROR_STREAM("Takeoff Param Number ERROR");
        }

    } else if (param[0] == "landing") {
        if (param.size() != 1) {
            ROS_ERROR_STREAM("Param Number ERROR");
            return;
        }
        if (!Landing()) ROS_ERROR_STREAM("Landing ERROR");

    } else if (param[0] == "returntohome") {
        if (current_move_check_flag) {
            ROS_ERROR_STREAM("busy!");
            return;
        }
        if (param.size() == 1) {
            // ROS_ERROR_STREAM("Param Number ERROR");
            std::thread thread_t1(&PXDroneControl::ReturnToHome, this, RTH_Altitude);
            thread_t1.detach();
        }
        if (param.size() == 2) {
            // ROS_ERROR_STREAM("Param Number ERROR");
            std::thread thread_t1(&PXDroneControl::ReturnToHome, this, stod(param[1]));
            thread_t1.detach();
        }
        ROS_INFO_STREAM("parents finish");
    } else if (param[0] == "movebypose") {
        if (param.size() != 5) {
            ROS_ERROR_STREAM("Param Number ERROR");
            return;
        }
        if (current_move_check_flag) {
            ROS_ERROR_STREAM("busy!");
            return;
        }
        Pre_Goal_Pose_[_X] = stod(param[1]);
        Pre_Goal_Pose_[_Y] = stod(param[2]);
        Pre_Goal_Pose_[_Z] = stod(param[3]);
        double_t tmp_th_goal = stod(param[4]);
        if (tmp_th_goal > 360.0) {
            ROS_ERROR_STREAM("input degree 0~360");
            return;
        }
        Pre_Yaw_Goal_ = DEG_TO_RAD(tmp_th_goal);
        TransRadTOPitoPi(Pre_Yaw_Goal_, Pre_Yaw_Goal_);
        std::thread thread_t1(&PXDroneControl::MoveToGoalVel, this);
        thread_t1.detach();
        ROS_INFO_STREAM("parents finish");
    } else if (param[0] == "navimodecheck") {
        ROS_INFO("Current mode = %s", navi_mode);
    } else if (param[0] == "navimodechange") {
        ROS_INFO("Current mode = %s", navi_mode);
        if (param[1] == "rtr") {
            navi_mode = "rtr";
        } else if (param[1] == "free") {
            navi_mode = "free";
        } else {
            ROS_ERROR_STREAM("param error");
            return;
        }
        ROS_INFO("Changed mode = %s", navi_mode);
    } else if (param[0] == "stop") {
        ROS_INFO_STREAM("stop");
        stop_flag = true;
    }

    else if (param[0] == "waypoint") {
        if (param.size() % 4 != 2) {
            ROS_ERROR_STREAM("incorrect param number");
            return;
        }
        if (current_move_check_flag) {
            ROS_ERROR_STREAM("busy!");
            return;
        }
        waypoint_vector.clear();
        Waypoint temp;
        for (uint8_t index = 1; index < param.size(); index++) {
            if (param[index] == "finish") break;
            if (index % 4 == 1) {
                temp.x_goal = stod(param[index]);
            } else if (index % 4 == 2) {
                temp.y_goal = stod(param[index]);
            } else if (index % 4 == 3) {
                temp.z_goal = stod(param[index]);
            } else if (index % 4 == 0) {
                temp.yaw_goal = stod(param[index]);
                double tmp_th_goal = temp.yaw_goal;
                if (tmp_th_goal > 360.0) {
                    ROS_ERROR_STREAM("input degree 0~360");
                    return;
                }
                temp.yaw_goal = DEG_TO_RAD(tmp_th_goal);
                TransRadTOPitoPi(temp.yaw_goal, temp.yaw_goal);
                waypoint_vector.emplace_back(temp);
                ROS_INFO("%lf %lf %lf %lf", temp.x_goal, temp.y_goal, temp.z_goal, temp.yaw_goal);
            }
        }
        std::thread thread_t2(&PXDroneControl::MoveWaypoint, this);
        thread_t2.detach();
        ROS_INFO_STREAM("parents finish");
    } else if (param[0] == "manual") {
        if (param.size() != 1) {
            ROS_INFO_STREAM("param number error");
            return;
        }
        std::thread thread_t4(&PXDroneControl::ManualControl, this);
        thread_t4.detach();
        ROS_INFO_STREAM("parents finish");
    } else {
        ROS_ERROR_STREAM("Incorrect CMD");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_subscriber");
    ros::NodeHandle priv_node("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    PXDroneControl px_drone_control;
    priv_node.param<std::string>("filter_type", px_drone_control.filter_type_, "EKF");

    priv_node.param<std::string>("marker_type", px_drone_control.marker_type_, "ARUCO");
    priv_node.param<bool>("acc_check", px_drone_control.acc_check_, false);
    priv_node.param<bool>("local_pose_check", px_drone_control.local_pose_check_, false);
    priv_node.param<bool>("body_vel_check", px_drone_control.body_vel_check_, false);
    priv_node.param<bool>("pose_estimation_check", px_drone_control.pose_estimation_check_, false);
    priv_node.param<double_t>("acc_threshold", px_drone_control.acc_threshold_, 0.08);

    priv_node.param<double_t>("RTH_Altitude", px_drone_control.RTH_Altitude, 5);
    priv_node.param<double_t>("Threshold_Distance", px_drone_control.Threshold_Distance, 0.5);

    ros::Rate rate(100);

    while (ros::ok() && !px_drone_control.current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("Connected");
    ros::Duration(2.0).sleep();
    ros::Duration duration;
    px_drone_control.pid_time_begin = ros::Time::now();
    while (ros::ok()) {
        if (!px_drone_control.Other_Task_Flag) {
            px_drone_control.Linear_Flight_Algorithm(px_drone_control.Yaw_Goal);
        }
        px_drone_control.Vel_msg.linear.x = px_drone_control.Body_Vel_Input[_X];
        px_drone_control.Vel_msg.linear.y = px_drone_control.Body_Vel_Input[_Y];
        px_drone_control.Vel_msg.linear.z = px_drone_control.Body_Vel_Input[_Z];
        px_drone_control.Vel_msg.angular.z = px_drone_control.Yaw_Rate_Input;
        px_drone_control.drone_velocity_pub.publish(px_drone_control.Vel_msg);
        px_drone_control.pid_time_begin = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }
    ros::waitForShutdown();

    return 0;
}
#endif  // !__PIX_VM_COMMAND_SUB__
