#ifndef __PIX_SENSOR_SUB__
#define __PIX_SENSOR_SUB__

#include <pix_library_ass.h>

#include "pix_control.h"

void PXDroneControl::LidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // ROS_INFO("%d", (int)(msg->angle_max - msg->angle_min));
    int count = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // ROS_INFO("%d", count);
    // ROS_INFO("I heard a laser scan %s[%d]:", msg.header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD_TO_DEG(msg->angle_min), RAD_TO_DEG(msg->angle_max));
    std::vector<std::vector<double_t>> scan_dis;
    std::vector<double_t> scan_left, scan_right, scan_center;
    for (int i = 0; i < count; i++) {
        float degree = RAD_TO_DEG(msg->angle_min + msg->angle_increment * i);
        float dis = msg->ranges[i];
        if (dis != INFINITY) {
            if (dis < 1.5) {
                Lidar_Emergency_Flag = true;
            }
            if (degree < 15 && degree > -15) {
                scan_center.emplace_back(dis);
                // ROS_INFO(": [%f, %f]", degree, msg->ranges[i]);
            } else if (degree > 15 && degree < 60) {
                scan_right.emplace_back(dis);
                // ROS_INFO(": [%f, %f]", degree, msg->ranges[i]);

            } else if (degree < -15 && degree > -60) {
                scan_left.emplace_back(dis);
                // ROS_INFO(": [%f, %f]", degree, msg->ranges[i]);
            }
        }
    }
    scan_dis.emplace_back(scan_left);
    scan_dis.emplace_back(scan_right);
    scan_dis.emplace_back(scan_center);
    for (int8_t i = 0; i < 3; i++) {
        auto n = scan_dis[i].size();
        float_t average = 0.0f;
        if (n != 0) {
            average = std::accumulate(scan_dis[i].begin(), scan_dis[i].end(), 0.0) / n;
        } else {
            average = 15.0;
        }
        Lidar_Scan_Dis[i] = average;
        // ROS_INFO("avr_dis[%d] : %f",i,average);
    }
}

void PXDroneControl::Imu_Callback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    TransRadTOPitoPi(roll, roll);
    TransRadTOPitoPi(pitch, pitch);
    TransRadTOPitoPi(yaw, yaw);

    if (Imu_Start_flag_cnt < 100) {
        Imu_yaw_vec.emplace_back(yaw);
        Imu_roll_vec.emplace_back(roll);
        Imu_pitch_vec.emplace_back(pitch);

        Acc_x_vec.emplace_back(((-1) * msg->linear_acceleration.x) + (msg->linear_acceleration.z * sin(pitch)));
        Acc_y_vec.emplace_back(msg->linear_acceleration.y - (msg->linear_acceleration.z * cos(pitch) * sin(roll)));
        Acc_z_vec.emplace_back(msg->linear_acceleration.z * cos(roll) * cos(pitch));
        ++Imu_Start_flag_cnt;
    } else if (Imu_Start_flag_cnt == 100) {
        std::sort(Acc_x_vec.begin(), Acc_x_vec.end());
        std::sort(Acc_y_vec.begin(), Acc_y_vec.end());
        std::sort(Acc_z_vec.begin(), Acc_z_vec.end());

        std::sort(Imu_yaw_vec.begin(), Imu_yaw_vec.end());
        std::sort(Imu_roll_vec.begin(), Imu_roll_vec.end());
        std::sort(Imu_pitch_vec.begin(), Imu_pitch_vec.end());
        double_t acc_std_x, acc_std_y, acc_std_z, acc_std;
        for (int i = 20; i < 80; i++) {
            yaw_abs += Imu_yaw_vec[i];
            roll_abs += Imu_roll_vec[i];
            pitch_abs += Imu_pitch_vec[i];

            Acc_Bias_x += Acc_x_vec[i];
            Acc_Bias_y += Acc_y_vec[i];
            Acc_Bias_z += Acc_z_vec[i];
        }
        Acc_Bias_x /= 60;
        Acc_Bias_y /= 60;
        Acc_Bias_z /= 60;

        yaw_abs /= 60;
        roll_abs /= 60;
        pitch_abs /= 60;

        for (int i = 20; i < 80; i++) {
            acc_std_x += pow((Acc_x_vec[i] - Acc_Bias_x), 2);
            acc_std_y += pow((Acc_y_vec[i] - Acc_Bias_y), 2);
            acc_std_z += pow((Acc_z_vec[i] - Acc_Bias_z), 2);
        }
        acc_std_x /= 60;
        acc_std_y /= 60;
        acc_std_z /= 60;

        acc_std = (acc_std_x + acc_std_y + acc_std_z) / 3.0;
        // Q_EKF = 10000000.0 * acc_std * Eigen::MatrixXf::Identity(6, 6);
        // Q_UKF = Q_EKF;
        ROS_INFO("acc_std : %f", acc_std);

        filter_begin = ros::Time::now();
        Imu_Start_flag_cnt++;
    } else {
        double_t filterd_acc_x, filterd_acc_y, filterd_acc_z;

        Body_RPY_Cur[_ROLL] = roll - roll_abs;
        TransRadTOPitoPi(Body_RPY_Cur[_ROLL], Body_RPY_Cur[_ROLL]);
        Body_RPY_Cur[_PITCH] = pitch - pitch_abs;
        TransRadTOPitoPi(Body_RPY_Cur[_PITCH], Body_RPY_Cur[_PITCH]);

        Body_RPY_Cur[_YAW] = yaw - yaw_abs;
        TransRadTOPitoPi(Body_RPY_Cur[_YAW], Body_RPY_Cur[_YAW]);
        // Body_RPY_Cur[_ROLL] = roll - roll_abs;
        // Body_RPY_Cur[_PITCH] = pitch - pitch_abs;

        // filterd_acc_x = Acc_LPF_Alpha * Body_Acc_Cur[_X] + (1 - Acc_LPF_Alpha) * (((-1) * msg->linear_acceleration.x) - Acc_Bias_x + (Acc_Bias_z * sin(Body_RPY_Cur[_PITCH])));
        // filterd_acc_y = Acc_LPF_Alpha * Body_Acc_Cur[_Y] + (1 - Acc_LPF_Alpha) * (msg->linear_acceleration.y - Acc_Bias_y - (Acc_Bias_z * cos(Body_RPY_Cur[_PITCH]) * sin(Body_RPY_Cur[_ROLL])));
        // filterd_acc_z = Acc_LPF_Alpha * Body_Acc_Cur[_Z] + (1 - Acc_LPF_Alpha) * (msg->linear_acceleration.z * cos(Body_RPY_Cur[_PITCH]) * cos(Body_RPY_Cur[_ROLL]) - Acc_Bias_z);
        Body_Acc_Cur[_X] = Acc_LPF_Alpha * Body_Acc_Cur[_X] + (1 - Acc_LPF_Alpha) * (((-1) * msg->linear_acceleration.x) - Acc_Bias_x + (Acc_Bias_z * sin(Body_RPY_Cur[_PITCH])));
        Body_Acc_Cur[_Y] = Acc_LPF_Alpha * Body_Acc_Cur[_Y] + (1 - Acc_LPF_Alpha) * (msg->linear_acceleration.y - Acc_Bias_y - (Acc_Bias_z * cos(Body_RPY_Cur[_PITCH]) * sin(Body_RPY_Cur[_ROLL])));
        Body_Acc_Cur[_Z] = Acc_LPF_Alpha * Body_Acc_Cur[_Z] + (1 - Acc_LPF_Alpha) * (msg->linear_acceleration.z * cos(Body_RPY_Cur[_PITCH]) * cos(Body_RPY_Cur[_ROLL]) - Acc_Bias_z);
        // Body_Acc_Cur[_X] = round(filterd_acc_x * 10) / 10;
        // Body_Acc_Cur[_Y] = round(filterd_acc_y * 10) / 10;
        // Body_Acc_Cur[_Z] = round(filterd_acc_z * 10) / 10;

        // SENSOR_ZERO_THRESHOLD(fabs(Body_Acc_Cur[_X]), acc_threshold_, Body_Acc_Cur[_X]);
        // SENSOR_ZERO_THRESHOLD(fabs(Body_Acc_Cur[_Y]), acc_threshold_, Body_Acc_Cur[_Y]);
        // SENSOR_ZERO_THRESHOLD(fabs(Body_Acc_Cur[_Z]), acc_threshold_, Body_Acc_Cur[_Z]);

        if (acc_check_) {
            ROS_INFO("acc %f, %f , %f", Body_Acc_Cur[_X], Body_Acc_Cur[_Y], Body_Acc_Cur[_Z]);
        }

        EKF_Filter();

        UKF_Filter();
        // }
        tf::Quaternion q;
        q.setRPY(Body_RPY_Cur[_ROLL], Body_RPY_Cur[_PITCH], Body_RPY_Cur[_YAW]);

        geometry_msgs::Quaternion odom_quat;
        tf::quaternionTFToMsg(q, odom_quat);

        geometry_msgs::PoseStamped temp;
        // temp.header.frame_id = "/base_link";
        temp.pose.position.x = EKF_Pose[_X];
        temp.pose.position.y = EKF_Pose[_Y];
        temp.pose.position.z = EKF_Pose[_Z];
        temp.pose.orientation.x = odom_quat.x;
        temp.pose.orientation.y = odom_quat.y;
        temp.pose.orientation.z = odom_quat.z;
        temp.pose.orientation.w = odom_quat.w;
        EKF_msgs.header.frame_id = "/base_link";
        EKF_msgs.poses.push_back(temp);
        EKF_pub.publish(EKF_msgs);

        temp.pose.position.x = UKF_Pose[_X];
        temp.pose.position.y = UKF_Pose[_Y];
        temp.pose.position.z = UKF_Pose[_Z];
        temp.pose.orientation.x = odom_quat.x;
        temp.pose.orientation.y = odom_quat.y;
        temp.pose.orientation.z = odom_quat.z;
        temp.pose.orientation.w = odom_quat.w;
        UKF_msgs.header.frame_id = "/base_link";
        UKF_msgs.poses.push_back(temp);
        UKF_pub.publish(UKF_msgs);
    }
}

void PXDroneControl::LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    const double_t mean = 0.0;
    const double_t stddev = 2.0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    double_t after_noise_x = msg->pose.position.x + dist(generator);
    double_t after_noise_y = msg->pose.position.y + dist(generator);
    double_t after_noise_z = msg->pose.position.z + dist(generator);

    if (Odom_Start_flag_cnt < 50) {
        Local_Odom_x_vec.emplace_back(after_noise_x);
        Local_Odom_y_vec.emplace_back(after_noise_y);
        Local_Odom_z_vec.emplace_back(after_noise_z);

        ++Odom_Start_flag_cnt;
    } else if (Odom_Start_flag_cnt == 50) {
        double_t sum_x, sum_y, sum_z;
        std::sort(Local_Odom_x_vec.begin(), Local_Odom_x_vec.end());
        std::sort(Local_Odom_y_vec.begin(), Local_Odom_y_vec.end());
        std::sort(Local_Odom_z_vec.begin(), Local_Odom_z_vec.end());
        double_t std_x, std_y, std_z, std_all;
        for (int i = 10; i < 40; i++) {
            sum_x += Local_Odom_x_vec[i];
            sum_y += Local_Odom_y_vec[i];
            sum_z += Local_Odom_z_vec[i];
        }

        GPS_Bias_x = sum_x / 30.0;
        GPS_Bias_y = sum_y / 30.0;
        GPS_Bias_z = sum_z / 30.0;

        for (int i = 10; i < 40; i++) {
            std_x += pow((Local_Odom_x_vec[i] - GPS_Bias_x), 2);
            std_y += pow((Local_Odom_y_vec[i] - GPS_Bias_y), 2);
            std_z += pow((Local_Odom_z_vec[i] - GPS_Bias_z), 2);
        }
        std_x /= 30;
        std_y /= 30;
        std_z /= 60;

        std_all = (std_x + std_y + std_z) / 3.0;
        ROS_INFO("std_GPS: %f", std_all);
        // R_EKF = 1000000.0 * std_all * Eigen::MatrixXf::Identity(6, 6);
        // R_UKF = R_EKF;

        ++Odom_Start_flag_cnt;

    } else {
        double tr_x = after_noise_x - GPS_Bias_x;
        double tr_y = after_noise_y - GPS_Bias_y;
        double tr_z = after_noise_z - GPS_Bias_z;

        Eigen::MatrixXf Tr(2, 2), Temp_pose(2, 1), RT_pose;
        double th;

        th = -(yaw_abs);
        Tr << cos(th), -sin(th),
            sin(th), cos(th);

        Temp_pose << tr_x, tr_y;
        RT_pose = Tr * Temp_pose;

        Z_EKF(0) = RT_pose(0);
        Z_EKF(1) = RT_pose(1);
        Z_EKF(2) = tr_z;

        Z_UKF(0) = RT_pose(0);
        Z_UKF(1) = RT_pose(1);
        Z_UKF(2) = tr_z;

        GPS_Pose[_X] = RT_pose(0);
        GPS_Pose[_Y] = RT_pose(1);
        GPS_Pose[_Z] = tr_z;
        // Gps_msgs.poses.
        tf::Quaternion q;
        q.setRPY(Body_RPY_Cur[_ROLL], Body_RPY_Cur[_PITCH], Body_RPY_Cur[_YAW]);

        geometry_msgs::Quaternion odom_quat;
        tf::quaternionTFToMsg(q, odom_quat);
        geometry_msgs::PoseStamped temp;

        temp.pose.position.x = GPS_Pose[_X];
        temp.pose.position.y = GPS_Pose[_Y];
        temp.pose.position.z = GPS_Pose[_Z];
        temp.pose.orientation.x = odom_quat.x;
        temp.pose.orientation.y = odom_quat.y;
        temp.pose.orientation.z = odom_quat.z;
        temp.pose.orientation.w = odom_quat.w;
        GPS_msgs.header.frame_id = "/base_link";
        GPS_msgs.poses.push_back(temp);
        GPS_pub.publish(GPS_msgs);

        if (local_pose_check_) {
            ROS_INFO("GPS_Pose %f, %f , %f", GPS_Pose[_X], GPS_Pose[_Y], GPS_Pose[_Z]);
        }
    }
}

void PXDroneControl::Body_Vel_Callback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    if (Body_Vel_Start_flag_cnt < 50) {
        Body_Xvel_vec.emplace_back(msg->twist.linear.x);
        Body_Yvel_vec.emplace_back(msg->twist.linear.y);
        Body_Zvel_vec.emplace_back(msg->twist.linear.z);

        ++Body_Vel_Start_flag_cnt;
    } else if (Body_Vel_Start_flag_cnt == 50) {
        std::sort(Body_Xvel_vec.begin(), Body_Xvel_vec.end());
        std::sort(Body_Yvel_vec.begin(), Body_Yvel_vec.end());
        std::sort(Body_Zvel_vec.begin(), Body_Zvel_vec.end());

        for (int i = 10; i < 40; i++) {
            Body_Vel_Measure_Bias[_X] += Body_Xvel_vec[i];
            Body_Vel_Measure_Bias[_Y] += Body_Yvel_vec[i];
            Body_Vel_Measure_Bias[_Z] += Body_Zvel_vec[i];
        }

        Body_Vel_Measure_Bias[_X] /= 30;
        Body_Vel_Measure_Bias[_Y] /= 30;
        Body_Vel_Measure_Bias[_Z] /= 30;

        ++Body_Vel_Start_flag_cnt;

    } else {
        double_t tmp_x_vel, tmp_y_vel, tmp_z_vel;
        tmp_x_vel = msg->twist.linear.x - Body_Vel_Measure_Bias[_X];
        tmp_y_vel = msg->twist.linear.y - Body_Vel_Measure_Bias[_Y];
        tmp_z_vel = msg->twist.linear.z - Body_Vel_Measure_Bias[_Z];

        tmp_x_vel = round(tmp_x_vel * 100) / 100;
        tmp_y_vel = round(tmp_y_vel * 100) / 100;
        tmp_z_vel = round(tmp_z_vel * 100) / 100;

        SENSOR_ZERO_THRESHOLD(fabs(tmp_x_vel), acc_threshold_, tmp_x_vel);
        SENSOR_ZERO_THRESHOLD(fabs(tmp_y_vel), acc_threshold_, tmp_y_vel);
        SENSOR_ZERO_THRESHOLD(fabs(tmp_z_vel), acc_threshold_, tmp_z_vel);

        Z_EKF(3) = tmp_x_vel * cos(Body_RPY_Cur[_YAW]) - tmp_y_vel * sin(Body_RPY_Cur[_YAW]);
        Z_EKF(4) = tmp_x_vel * sin(Body_RPY_Cur[_YAW]) + tmp_y_vel * cos(Body_RPY_Cur[_YAW]);
        Z_EKF(5) = tmp_z_vel;

        Z_UKF(3) = tmp_x_vel * cos(Body_RPY_Cur[_YAW]) - tmp_y_vel * sin(Body_RPY_Cur[_YAW]);
        Z_UKF(4) = tmp_x_vel * sin(Body_RPY_Cur[_YAW]) + tmp_y_vel * cos(Body_RPY_Cur[_YAW]);
        Z_UKF(5) = tmp_z_vel;

        if (body_vel_check_) {
            ROS_INFO("body vel %f, %f , %f", tmp_x_vel, tmp_y_vel, tmp_z_vel);
        }
    }
}

void PXDroneControl::GoundtruthPath(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseStamped temp;
    temp.pose.orientation.x = msg->pose.pose.orientation.x;
    temp.pose.orientation.y = msg->pose.pose.orientation.y;
    temp.pose.orientation.z = msg->pose.pose.orientation.z;
    temp.pose.orientation.w = msg->pose.pose.orientation.w;
    temp.pose.position.x = msg->pose.pose.position.x;
    temp.pose.position.y = msg->pose.pose.position.y;
    temp.pose.position.z = msg->pose.pose.position.z;

    Ground_Truth_msgs.header.frame_id = "/base_link";
    Ground_Truth_msgs.poses.push_back(temp);
    Ground_Truth_pub.publish(Ground_Truth_msgs);
}

#endif
