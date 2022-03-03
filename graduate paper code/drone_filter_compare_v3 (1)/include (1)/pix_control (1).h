#ifndef __PIX_VM_COMMAND_SUB_H__
#define __PIX_VM_COMMAND_SUB_H__

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <stdio.h>
#include <termios.h>
#include <tf/tf.h>
#include <tf2/convert.h>
#include <unistd.h>

#include <../src/pix_pid_control_library.cpp>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "pix_pid_control_library.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
// #include "vm_pixhawk/KeyboardMsg.h"

#define RAD_TO_DEG(RAD) (RAD) * (180.f) / (M_PI)
#define DEG_TO_RAD(DEG) (DEG) * (M_PI) / (180.f)
#define RAD_360 (2 * M_PI)
#define SENSOR_ZERO_THRESHOLD(X, Y, Z) ((X < Y) ? (Z = 0) : 1)

enum _Info_UKF_Type {
    _UKF_X,
    _UKF_Z,
};

enum _Info_Dir_Type {
    _X,
    _Y,
    _Z,
};
enum _Info_RPY_Type {
    _ROLL,
    _PITCH,
    _YAW,
};

enum _Basic_Move_Type {
    _ROTATE_,
    _LINEAR_MOVE_,
    _FINAL_ROTATE_,
};

class PXDroneControl {
   private:
    ros::NodeHandle nh;
    double_t Acc_Bias_x = 0, Acc_Bias_y = 0, Acc_Bias_z = 0, Acc_LPF_Alpha = 0.2;
    double_t GPS_Bias_x = 0, GPS_Bias_y = 0, GPS_Bias_z = 0;
    double_t yaw_abs = 0, roll_abs = 0, pitch_abs = 0;

   protected:
    struct Waypoint {
        double x_goal, y_goal, z_goal, yaw_goal;
    };
    std::vector<Waypoint> waypoint_vector;
    Eigen::Vector3d Goal_pose, Pre_Goal_Pose_,
        EKF_Vel, Body_RPY_Cur, Body_Acc_Cur, Estimated_Pose,
        EKF_Pose, UKF_Pose, GPS_Pose, Estimated_Vel, Body_Vel_Measure_Bias;
    uint16_t Imu_Start_flag_cnt = 0, Odom_Start_flag_cnt = 0, Body_Vel_Start_flag_cnt = 0;
    std::vector<double_t> Imu_yaw_vec,
        Acc_x_vec, Acc_y_vec, Acc_z_vec,
        Local_Odom_x_vec, Local_Odom_y_vec, Local_Odom_z_vec,
        Body_Xvel_vec, Body_Yvel_vec, Body_Zvel_vec,
        Imu_roll_vec, Imu_pitch_vec;
    Eigen::MatrixXf A_EKF, Q_EKF, R_EKF, H_EKF, X_EKF, Z_EKF, P_EKF, Tr, Xp_EKF, Pp_EKF, K_EKF, Q_EKF_Init, R_EKF_Init;
    Eigen::MatrixXf A_UKF, Q_UKF, R_UKF, H_UKF, X_UKF, Z_UKF, P_UKF, Xp_UKF, Pp_UKF, K_UKF, Pz_UKF, Zp_UKF, Xi, W;
    Eigen::Vector3d Ideal_Pose;
    Eigen::MatrixXf Adaptive_P_UKF;
    uint8_t X_n, X_m;
    double_t Plane_Speed = 0, Turn_Speed = 0, Z_Speed = 0;
    bool current_move_check_flag = false;
    double_t Pre_Yaw_Goal_ = 0.0;
    double_t Ground_Truth_Cnt = 0;
    nav_msgs::Path UKF_msgs, GPS_msgs, EKF_msgs, Ground_Truth_msgs;
    double_t gt_bias_x = 0, gt_bias_y = 0, gt_bias_z = 0;
    uint8_t UKF_Number = 1;
    uint8_t tmp_cnt = 0;

   public:
    PXDroneControl();
    ~PXDroneControl();
    std::string navi_mode = "free";
    double_t pid_dt = 0.01, pid_max = 0.7, pid_min = 0.0, pid_Kp = 0.7, pid_Kd = 0.5, pid_Ki = 0.0;
    double_t pid_z_dt = 0.01, pid_z_max = 1.0, pid_z_min = 0.0, pid_z_Kp = 0.7, pid_z_Kd = 0.1, pid_z_Ki = 0.0;
    double_t pid_th_dt = 0.01, pid_th_max = 1.0, pid_th_min = 0.0, pid_th_Kp = 0.5, pid_th_Kd = 0.5, pid_th_Ki = 0.0;
    std::string filter_type_, marker_type_;
    bool acc_check_, local_pose_check_, body_vel_check_, pose_estimation_check_;
    double_t acc_threshold_, RTH_Altitude, Threshold_Distance, Threshold_Distance_free_mode = 0.8;
    ros::Time Server_Watchdog_, filter_begin;
    geometry_msgs::Twist Vel_msg;
    mavros_msgs::State current_state;
    mavros_msgs::SetMode set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandTOL takeoff_land_msgs;
    nav_msgs::Odometry odom_msgs;

    ros::Publisher drone_velocity_pub, UKF_pub, GPS_pub, EKF_pub, Ground_Truth_pub;
    ros::Subscriber drone_state_sub, drone_control_sub, drone_imu_sub,
        drone_body_vel_sub, drone_local_position_sub, drone_lidar_sub,
        watchdog_sub, ground_truth_sub;
    ros::ServiceClient drone_land_client, drone_arm_client, drone_setmode_client, drone_takeoff_client;

    bool stop_flag = false;
    ros::Time pid_time_begin;
    Eigen::Vector3d Body_Vel_Input;
    double_t Yaw_Rate_Input;
    double_t Yaw_Goal = 0.0;
    // bool Vel_Mission_flag=false;
    PID linear_speed_pid;
    PID th_w_pid;
    PID z_linear_speed_pid;

    bool Other_Task_Flag = false;
    bool Lidar_Emergency_Flag = false;
    bool Arrive_Flag = false;
    bool Satisfy_Yaw_Flag = false;

    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void CommandCallback(const std_msgs::String::ConstPtr &msg);
    bool Take_Off(double_t altitude = 3, double_t takeoff_vel = 0.5);
    bool Landing(void);
    void ReturnToHome(double_t altitude);
    bool Init_Set(void);
    void Imu_Callback(const sensor_msgs::Imu::ConstPtr &msg);
    void Body_Vel_Callback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void EKF_Filter(void);
    void SigmaPoint(Eigen::MatrixXf &X, Eigen::MatrixXf &P, float kappa);
    void UT(Eigen::MatrixXf &Xi, Eigen::MatrixXf &W, Eigen::MatrixXf &noiseCov, uint8_t type);
    void UKF_Filter(void);
    void LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    bool MoveToGoalVel(void);
    void Two_Point_Distance(const std::vector<std::pair<double, double>> &point, double_t &distance);
    void Distance(const std::vector<std::pair<double, double>> &point, double_t &distance);
    double Linear_Flight_Algorithm(double_t &goal_yaw, double_t threshold = DEG_TO_RAD(3.0));
    int getch(void);
    Eigen::MatrixXf CalFunction(Eigen::MatrixXf xhat);
    void MoveWaypoint(void);
    void ManualControl(void);
    bool RotateAlgorithm(double_t &goal_yaw, double_t &yaw, double_t threshold = DEG_TO_RAD(3.0));

    void TransRadTOPitoPi(double_t &before_rad, double_t &after_rad);
    void ZeroVelocity(void);
    void GoundtruthPath(const nav_msgs::Odometry::ConstPtr &msg);
};
PXDroneControl::PXDroneControl() {
    /**pub**/
    Init_Set();
    drone_velocity_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    UKF_pub = nh.advertise<nav_msgs::Path>("/UKF_path", 10);
    EKF_pub = nh.advertise<nav_msgs::Path>("/EKF_path", 10);
    GPS_pub = nh.advertise<nav_msgs::Path>("/GPS_path", 10);
    Ground_Truth_pub = nh.advertise<nav_msgs::Path>("/Ground_Truth", 10);
    /**sub**/
    ground_truth_sub = nh.subscribe("/odom", 1, &PXDroneControl::GoundtruthPath, this);
    drone_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &PXDroneControl::state_cb, this);
    drone_imu_sub = nh.subscribe("/mavros/imu/data", 1, &PXDroneControl::Imu_Callback, this);
    drone_body_vel_sub = nh.subscribe("/mavros/local_position/velocity_body", 1, &PXDroneControl::Body_Vel_Callback, this);
    drone_setmode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    drone_arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    drone_land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    drone_takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    drone_local_position_sub = nh.subscribe("/mavros/local_position/pose", 1, &PXDroneControl::LocalPositionCallback, this);
    drone_control_sub = nh.subscribe("keyboard_command", 1, &PXDroneControl::CommandCallback, this);
    // ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
}

PXDroneControl::~PXDroneControl() {
}

#endif
