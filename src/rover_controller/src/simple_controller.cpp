#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>


using std::placeholders::_1;

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


////////////////////////////////////////////////////////////////////////////////////////
#include <chrono>
#include <memory>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"


#include "sensor_msgs/msg/imu.hpp"

// #include "sensor_msgs/msg/joint_state.hpp"
// #include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
// #include "tf2_ros/transform_broadcaster.h"

class SimpleController : public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        // Odometry
        // double wheel_radius_;
        // double wheel_separation_;
        // Eigen::Matrix2d speed_conversion_;
        double right_wheel_prev_pos_;
        double left_wheel_prev_pos_;
        rclcpp::Time prev_time_;
        double x_;
        double y_;
        double theta_;

        // TF
        std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

        #define ROVER_WHEEL_RADIUS 0.075

        #define d1 0.177 // Distance from the center to the front-left wheel along the x-axis
        #define d2 0.310 // Distance from the center to the front-left wheel along the y-axis
        #define d3 0.274 // Distance from the center to the front-rear wheels along the y-axis
        #define d4 0.253 // Distance from the center to the center wheel along the x-axis
        double theta = 0;
        double l = 0; // linear, angular turning radius

        double theta_front_closest;
        double theta_front_farthest;

        double angular_velocity_center, vel_middle_closest, vel_corner_closest, vel_corner_farthest, vel_middle_farthest;
        double ang_vel_middle_closest, ang_vel_corner_closest, ang_vel_corner_farthest, ang_vel_middle_farthest;
        double start_time, time, pre_time;

        geometry_msgs::msg::Twist pri_velocity;


        double fl_vel, fr_vel, ml_vel, mr_vel, rl_vel, rr_vel;
        double current_dl, dl, pre_dl;
        double x_postion, y_postion;

        double FL_data, FR_data, ML_data, MR_data, RL_data, RR_data;

        double FR_servo_data, FL_servo_data, RR_servo_data, RL_servo_data;

        bool delay_ = true;
        nav_msgs::msg::Odometry odom_msg;

public:
    SimpleController(const std::string& name)
                                    : Node(name)
                                    , left_wheel_prev_pos_(0.0)
                                    , right_wheel_prev_pos_(0.0)
                                    , x_(0.0)
                                    , y_(0.0)
                                    , theta_(0.0)
    {
        wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 1);
        servo_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/servo_controller/joint_trajectory", 1);
        vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/rover_controller/cmd_vel", 1, std::bind(&SimpleController::velCallback, this, _1));
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&SimpleController::jointStateCallback, this, _1));
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rover_controller/odom", 10);
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/out", 1, std::bind(&SimpleController::imuCallback, this, std::placeholders::_1));

        transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // transform_stamped_.header.frame_id = "odom";
        // transform_stamped_.child_frame_id = "base_footprint";

        prev_time_ = get_clock()->now();
    }

    //whenever a new velocity command (message) is received this callback function is called
    //joystick publishes the velocity command in the form of a TwistStamped message
    //This callback function converts the velocity command to wheel speeds
    //and publishes to the wheel controller
    //the wheel controller is a separate node that takes care of the low level control of the wheels
    void velCallback(const geometry_msgs::msg::TwistStamped &msg)
    {
        if((pri_velocity.linear.x == msg.twist.linear.x) && (pri_velocity.angular.z == msg.twist.angular.z)) return;

        if(delay_)
        {
            delay_ = false;

            if (msg.twist.angular.z == 0 && msg.twist.linear.x != 0) { // linear velocity
                go_straight(msg);
                publishVelocity();
                publishAngles();
            }

            else if((msg.twist.angular.z != 0)&&(msg.twist.linear.x == 0))  // rotate in place
            {
                FL_servo_data = -atan(d3/d1);
                FR_servo_data = atan(d3/d1);
                RL_servo_data = atan(d2/d1);
                RR_servo_data = -atan(d2/d1);

                rotate_in_place(msg);
                publishAngles();                // servo angle pub
                publishVelocity();
            }

            else if((msg.twist.angular.z != 0)&&(msg.twist.linear.x != 0)) // rotation
            {
                // 1. Calculate turning radius
                twist_to_turning_radius(msg);

                // 2. Calculate servo motor angle
                calculate_servo_angle(l);

                // 3. Calculate wheel angular velocity
                calculate_drive_velocity(msg.twist.linear.x, l);

                // 4. publish
                publishAngles();
                publishVelocity();
            }

            else if((msg.twist.angular.z == 0)&&(msg.twist.linear.x == 0))
            {
                stop();
                publishAngles();
                publishVelocity();
            }

            delay_ = true;
        }

        pri_velocity.linear.x = msg.twist.linear.x;
        pri_velocity.angular.z = msg.twist.angular.z;
    }

    void calculate_servo_angle(double l)
    {
        theta_front_closest = atan2(d3, abs(l) - d1);
        theta_front_farthest = atan2(d3, abs(l) + d1);

        if(l > 0)
        {
            FL_servo_data = theta_front_closest;
            FR_servo_data = theta_front_closest;
            RL_servo_data = -theta_front_closest;
            RR_servo_data = -theta_front_closest;
        }
        else
        {
            FL_servo_data = -theta_front_closest;
            FR_servo_data = -theta_front_closest;
            RL_servo_data = theta_front_closest;
            RR_servo_data = theta_front_closest;
        }

    }

    void calculate_drive_velocity(float velocity, double l)
    {
        angular_velocity_center = velocity / abs(l);

        vel_middle_closest = (abs(l) - d4) * angular_velocity_center;
        vel_corner_closest = hypot(abs(l) - d1, d3) * angular_velocity_center;
        vel_corner_farthest = hypot(abs(l) + d1, d3) * angular_velocity_center;
        vel_middle_farthest = (abs(l) + d4) * angular_velocity_center;

        ang_vel_middle_closest = vel_middle_closest / ROVER_WHEEL_RADIUS;
        ang_vel_corner_closest = vel_corner_closest / ROVER_WHEEL_RADIUS;
        ang_vel_corner_farthest = vel_corner_farthest / ROVER_WHEEL_RADIUS;
        ang_vel_middle_farthest = vel_middle_farthest / ROVER_WHEEL_RADIUS;

        if (l > 0)  // turning left
        {
            FL_data = float(ang_vel_corner_closest);
            RL_data = float(ang_vel_corner_closest);

            ML_data = float(ang_vel_middle_closest);
            FR_data = float(ang_vel_corner_farthest);

            RR_data = float(ang_vel_corner_farthest);
            MR_data = float(ang_vel_middle_farthest);
        }
        else        // turning right
        {
            FL_data = float(ang_vel_corner_farthest);
            RL_data = float(ang_vel_corner_farthest);

            ML_data = float(ang_vel_middle_farthest);
            FR_data = float(ang_vel_corner_closest);

            RR_data = float(ang_vel_corner_closest);
            MR_data = float(ang_vel_middle_closest);

        }

    }

    void stop()
    {
        FL_data = 0;
        RL_data = 0;

        ML_data = 0;
        FR_data = 0;

        RR_data = 0;
        MR_data = 0;


        FL_servo_data = 0;
        FR_servo_data = 0;
        RL_servo_data = 0;
        RR_servo_data = 0;
    }

    void go_straight(const geometry_msgs::msg::TwistStamped &msg) {
        double velocity_data = msg.twist.linear.x / ROVER_WHEEL_RADIUS;

        FL_data = velocity_data;
        FR_data = velocity_data;
        ML_data = velocity_data;
        MR_data = velocity_data;
        RL_data = velocity_data;
        RR_data = velocity_data;

        FL_servo_data = 0;
        FR_servo_data = 0;
        RL_servo_data = 0;
        RR_servo_data = 0;

    }

    void twist_to_turning_radius(const geometry_msgs::msg::TwistStamped &msg)
    {
        l = msg.twist.linear.x / msg.twist.angular.z;
    }

    void rotate_in_place(const geometry_msgs::msg::TwistStamped &msg)
    {
        FL_data = -float(sqrt(d1*d1+d3*d3) * msg.twist.angular.z / ROVER_WHEEL_RADIUS);
        RL_data = -float(sqrt(d1*d1+d2*d2) * msg.twist.angular.z / ROVER_WHEEL_RADIUS);

        ML_data = -float(d4 * msg.twist.angular.z / ROVER_WHEEL_RADIUS);
        FR_data = float(sqrt(d1*d1+d3*d3) * msg.twist.angular.z / ROVER_WHEEL_RADIUS);

        RR_data = float(d4 * msg.twist.angular.z / ROVER_WHEEL_RADIUS);
        MR_data = float(sqrt(d1*d1+d2*d2) * msg.twist.angular.z / ROVER_WHEEL_RADIUS);
    }

    void publishVelocity() {

        std_msgs::msg::Float64MultiArray wheel;

        wheel.data = {ML_data, MR_data,
                        FL_data, FR_data,
                        RL_data, RR_data};

        wheel_cmd_pub_->publish(wheel);
    }

    void publishAngles()
    {

        auto servo = trajectory_msgs::msg::JointTrajectory();
        servo.joint_names = {"front_wheel_joint_R", "front_wheel_joint_L", "rear_wheel_joint_R", "rear_wheel_joint_L"};

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = {FR_servo_data, FL_servo_data,
                        RR_servo_data, RL_servo_data};
        point.velocities = {0.0, 0.0, 0.0, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(0.2);

        servo.points.push_back(point);
        servo_pub_->publish(servo);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg_state)
    {

        fl_vel = msg_state->position[5];
        fr_vel = msg_state->position[7];
        ml_vel = msg_state->position[2];
        mr_vel = msg_state->position[3];
        rl_vel = msg_state->position[8];
        rr_vel = msg_state->position[9];

        Odometry(theta);
    }

    void Odometry(double angle)
    {
        current_dl = (fl_vel + fr_vel + ml_vel + mr_vel + rl_vel + rr_vel) * ROVER_WHEEL_RADIUS / 6;
        dl = current_dl - pre_dl;

        pre_dl = current_dl;

        x_postion += dl * cos(angle);
        y_postion += dl * sin(angle);
        RCLCPP_INFO(this->get_logger(), "x_position: %f, y_position: %f", x_postion, y_postion);
        RCLCPP_INFO(this->get_logger(), "****angle: %f", angle);
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = x_postion;
        odom_msg.pose.pose.position.y = y_postion;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, angle);

        odom_msg.pose.pose.orientation.x = quaternion.x();
        odom_msg.pose.pose.orientation.y = quaternion.y();
        odom_msg.pose.pose.orientation.z = quaternion.z();
        odom_msg.pose.pose.orientation.w = quaternion.w();

        static tf2_ros::TransformBroadcaster transform_broadcaster(this);
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = x_postion;
        transform_stamped.transform.translation.y = y_postion;
        transform_stamped.transform.translation.z = 0.0;

        transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;
        transform_broadcaster.sendTransform(transform_stamped);

        odom_pub_->publish(odom_msg);

       // printf("x_pos : %f\ty_pos : %f\n", x_postion,y_postion);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->orientation, quaternion);

        // Convert quaternion to roll, pitch, yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(quaternion);
        m.getRPY(roll, pitch, yaw);
        theta = yaw;

    }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleController>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}