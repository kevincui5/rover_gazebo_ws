#include <Eigen/Geometry>
#include <Eigen/Core>


using std::placeholders::_1;

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <limits>
#include <unordered_map>


#include <chrono>
#include <memory>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"


#include "sensor_msgs/msg/imu.hpp"


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2_ros/transform_listener.h"
#include <cmath>  // for std::hypot
/* r -- turning radius.  A small value means a sharp turn, a large value means a wide turn. Any radius larger than max_radius will be considered a straight line because of servo motor encoder's resolution.
    WHEEL_RADIUS -- wheel radius
   d1 -- distance from the center to the front wheel along the x-axis 
   d2 -- distance from the center to the front wheel along the y-axis
   d3 -- distance from the center to the rear wheel along the y-axis
    d4 -- distance from the center to the middle wheel along the x-axis
*/
#define r_MAX 6.4 //meter
#define r_MIN 0.45 //meter
#define WHEEL_RADIUS 0.075


#define d1 0.177 
#define d2 0.310 
#define d3 0.274 
#define d4 0.253 

// #define NO_CMD_THRESHOLD 0.0 // threshold for no command in radians
#define DRIVE_NO_LOAD_RPM 223.0 //no load speed for the drive motors. NOTE: needs to be a float value

typedef struct WheelServoCommand {
    double vel_fl;
    double vel_fr;
    double vel_ml;
    double vel_mr;
    double vel_rl;
    double vel_rr;
    double angle_fl;
    double angle_fr;
    double angle_rl;
    double angle_rr;
} WheelServoCommand_;


class SimpleController : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    static constexpr double VEL_MAX_ = WHEEL_RADIUS * DRIVE_NO_LOAD_RPM * M_PI / 30.0; // max speed in m/s;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TwistWithCovariance curr_twist_ = geometry_msgs::msg::TwistWithCovariance();

public:
    SimpleController(const std::string& name)
                                    : Node(name)
                                    // , left_wheel_prev_pos_(0.0)
                                    // , right_wheel_prev_pos_(0.0)
                                    // , x_(0.0)
                                    // , y_(0.0)
                                    // , theta_(0.0)
    {
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/rover_controller/cmd_vel", 1, std::bind(&SimpleController::cmd_velCallback, this, _1));
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&SimpleController::jointStateCallback, this, _1));
        wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 1);
        servo_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/servo_controller/joint_trajectory", 1);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rover_controller/odom", 10);
        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     "imu/out", 1, std::bind(&SimpleController::imuCallback, this, std::placeholders::_1));

        transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_msg_.header.stamp = this->get_clock()->now();
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_footprint";
        odom_msg_.pose.pose.orientation.w = 1;
        // transform_stamped_.header.frame_id = "odom";
        // transform_stamped_.child_frame_id = "base_footprint";
        // ServoCommand servo_cmd;
    }

    //whenever a new velocity command (message) is received this callback function is called
    //joystick publishes the velocity command in the form of a TwistStamped message
    //This callback function converts the velocity command to wheel speeds
    //and publishes to the wheel controller
    //the wheel controller is a separate node that takes care of the low level control of the wheels
    void cmd_velCallback(const geometry_msgs::msg::TwistStamped &msg_twist)
    {
        bool is_intuitive = false;
        // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x: %f, angular.z: %f", msg_twist.twist.linear.x, msg_twist.twist.angular.z);
        
        // 1. Calculate the turning radius
        // 2. Calculate the servo motor angle
        // 3. Calculate the wheel angular velocity
        // 4. publish

        // WheelServoCommand_ command;
        
        // command = rotate_in_place(msg_twist);
    
        WheelServoCommand_ command;
        if((msg_twist.twist.angular.z != 0)&&(msg_twist.twist.linear.x == 0))  // rotate in place
        {
            command = rotate_in_place(msg_twist);
            publishCommand(command);   
        }
        else 
        {
            // 1. Calculate turning radius
            double r_desired = twist_to_turning_radius(msg_twist);

            // 2. Calculate servo motor angle
            auto servo_cmd = calculate_servo_angle(r_desired);
            double max_vel = abs(r_desired) / (abs(r_desired) + d1) * VEL_MAX_;
            if (isinf(max_vel)) {
                max_vel = VEL_MAX_;
            }
            double velocity = std::min(max_vel, msg_twist.twist.linear.x);
            // 3. Calculate wheel angular velocity
            auto wheel_cmd = calculate_wheel_velocity(velocity, r_desired);

            // 4. publish
            command.vel_fl = wheel_cmd[0];
            command.vel_fr = wheel_cmd[1];
            command.vel_ml = wheel_cmd[2];
            command.vel_mr = wheel_cmd[3];
            command.vel_rl = wheel_cmd[4];
            command.vel_rr = wheel_cmd[5];
            command.angle_fl = servo_cmd[0];
            command.angle_fr = servo_cmd[1];
            command.angle_rl = servo_cmd[2];
            command.angle_rr = servo_cmd[3];
            publishCommand(command);
        }
    }

    /* * Calculate the required angles of the 4 servo(corner) motors based on the turning radius
    A positive servo angle means turning right, a negative servo angle means turning left
    */
    static std::array<double, 4> calculate_servo_angle(double r)
    {
        std::array<double, 4> result = {0.0, 0.0, 0.0, 0.0};
        if (r > r_MAX) {
            return result;
        } 

        double theta_fl = atan2(d3, abs(r) - d1);
        double theta_fr = atan2(d3, abs(r) + d1);
        double theta_rl = atan2(d2, abs(r) - d1);
        double theta_rr = atan2(d2, abs(r) + d1);

        if(r > 0)
        {
            result[0] = -theta_fl;
            result[1] = -theta_fr;
            result[2] = theta_rl;
            result[3] = theta_rr;
        }
        else
        {
            result[0] = theta_fr;
            result[1] = theta_fl;
            result[2] = -theta_rr;
            result[3] = -theta_rl;
        }
        return result;
    }

    static std::array<double, 6> calculate_wheel_velocity(double velocity, double r)
    {
        std::array<double, 6> result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        velocity = std::max(-VEL_MAX_, std::min(VEL_MAX_, velocity));
        if (velocity == 0) {
            return result;
        } else if (std::abs(r) >= r_MAX) {
            double angular_vel = velocity / WHEEL_RADIUS;
            result[0] = angular_vel;
            result[1] = -angular_vel;
            result[2] = angular_vel;
            result[3] = -angular_vel;
            result[4] = angular_vel;
            result[5] = -angular_vel; 
        }
        else {
            r = std::abs(r);
            double angular_velocity_center = velocity / r;

            double vel_fl = hypot(abs(r) - d1, d3) * angular_velocity_center;
            double vel_fr = hypot(abs(r) + d1, d3) * angular_velocity_center;
            double vel_ml = (abs(r) - d4) * angular_velocity_center;
            double vel_mr = (abs(r) + d4) * angular_velocity_center;
            double vel_rl = hypot(abs(r) - d1, d2) * angular_velocity_center;
            double vel_rr = hypot(abs(r) + d1, d2) * angular_velocity_center;

            double ang_vel_fl = vel_fl / WHEEL_RADIUS;
            double ang_vel_fr = vel_fr / WHEEL_RADIUS;
            double ang_vel_ml = vel_ml / WHEEL_RADIUS;
            double ang_vel_mr = vel_mr / WHEEL_RADIUS;
            double ang_vel_rl = vel_rl/ WHEEL_RADIUS;
            double ang_vel_rr = vel_rr / WHEEL_RADIUS;

            if (r > 0)  // turning left
            {
                result[0] = float(ang_vel_fl);
                result[1] = float(ang_vel_fr);
                result[2] = float(ang_vel_ml);
                result[3] = float(ang_vel_mr);
                result[4] = float(ang_vel_rl);
                result[5] = float(ang_vel_rr);
            }
            else        // turning right
            {
                result[0] = float(ang_vel_fr);
                result[1] = float(ang_vel_fl);
                result[2] = float(ang_vel_ml);
                result[3] = float(ang_vel_mr);
                result[4] = float(ang_vel_rr);
                result[5] = float(ang_vel_rl);

            }
        }
        return result;
    }

    static double twist_to_turning_radius(const geometry_msgs::msg::TwistStamped &twist_msg, bool is_clipped = true, bool is_intuitive = false)
    {
        /*
        Convert the linear and angular velocity to a turning radius
        Turning left and positive angle corresponds to a positive turning radius

        :param twist_msg: geometry_msgs::msg::TwistStamped
        :return: turning radius for the given angle in [m]
        */
    
        double infinity = std::numeric_limits<double>::infinity();
        if (twist_msg.twist.angular.z == 0) {
            
            return infinity; // No turning radius if angular velocity is zero
        }
        double turning_radius = twist_msg.twist.linear.x / twist_msg.twist.angular.z;
        if (!is_clipped) {
            return turning_radius;
        }
        if (turning_radius == 0) {
            if (is_intuitive) {
                if (twist_msg.twist.linear.x == 0) {
                    return r_MAX; 
                } else {
                    turning_radius = r_MIN * VEL_MAX_ / twist_msg.twist.angular.z; 
                }
            } else {
                return r_MAX;
            }
        } 
        if (turning_radius > 0) {
            turning_radius = std::max(r_MIN, std::min(r_MAX, turning_radius)); 
        } 
        else {
            turning_radius = std::max(-r_MAX, std::min(-r_MIN, turning_radius)); 
        }
        return turning_radius;
    }

    static double angle_to_turning_radius(double angle, bool is_front)
    {
        /*
        Convert the angle of a virtual wheel positioned in the middle of the front two wheels to a turning radius
        Turning left and positive angle corresponds to a positive turning radius

        :param angle: [-pi/4, pi/4]
        :return: turning radius for the given angle in [m]
        */
        if (tan(angle) == 0) {
            return std::numeric_limits<double>::infinity(); // No turning radius if angle is zero
        }
        double distance = is_front ? d3 : d2;
        double turning_radius = distance / tan(angle);
        return turning_radius;
    }
    
    static WheelServoCommand_ rotate_in_place(const geometry_msgs::msg::TwistStamped &msg)
    {
        WheelServoCommand_ wheel_servo_cmd;
        // Calculate the corner/servo angles for rotating in place
        //the angle only depends on the ratio of d3 to d1 and d2
        wheel_servo_cmd.angle_fl = atan(d3 / d1);   
        wheel_servo_cmd.angle_rl = -wheel_servo_cmd.angle_fl;
        wheel_servo_cmd.angle_rr = atan(d2 / d1); 
        wheel_servo_cmd.angle_fr = -wheel_servo_cmd.angle_rr;
        // Calculate the wheel speeds for rotating in place
        double front_wheel_vel = std::hypot(d1, d3) * msg.twist.angular.z / WHEEL_RADIUS;
        double rear_wheel_vel = std::hypot(d1, d2) * msg.twist.angular.z / WHEEL_RADIUS;
        double middle_wheel_vel = d4 * msg.twist.angular.z / WHEEL_RADIUS;
        wheel_servo_cmd.vel_fl = front_wheel_vel;
        wheel_servo_cmd.vel_fr = front_wheel_vel;
        wheel_servo_cmd.vel_ml = middle_wheel_vel;
        wheel_servo_cmd.vel_mr = middle_wheel_vel;
        wheel_servo_cmd.vel_rl = rear_wheel_vel;
        wheel_servo_cmd.vel_rr = rear_wheel_vel;

        return wheel_servo_cmd;
    }


    void publishCommand(WheelServoCommand_& wheel_servo_cmd) {
        std_msgs::msg::Float64MultiArray wheel_msg;
        wheel_msg.data = {wheel_servo_cmd.vel_ml, wheel_servo_cmd.vel_mr, wheel_servo_cmd.vel_fl, wheel_servo_cmd.vel_fr, wheel_servo_cmd.vel_rl, wheel_servo_cmd.vel_rr};
        wheel_cmd_pub_->publish(wheel_msg);

        auto servo = trajectory_msgs::msg::JointTrajectory();
        servo.joint_names = {"front_wheel_joint_R", "front_wheel_joint_L", "rear_wheel_joint_R", "rear_wheel_joint_L"};

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = {wheel_servo_cmd.angle_fr, wheel_servo_cmd.angle_fl,
            wheel_servo_cmd.angle_rr, wheel_servo_cmd.angle_rl};
        point.velocities = {0.0, 0.0, 0.0, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(0.2);

        servo.points.push_back(point);
        servo_cmd_pub_->publish(servo);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg_state)
    {
        //see the documentation for the JointState message
        //https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/JointState.html
        auto joints_pos = std::make_unique<std::unordered_map<std::string, double>>();
        auto joints_vel = std::make_unique<std::unordered_map<std::string, double>>();
        for (size_t i = 0; i < msg_state->name.size(); ++i) {
            (*joints_pos)[msg_state->name[i]] = msg_state->position[i];
        }
        for (size_t i = 0; i < msg_state->name.size(); ++i) {
            (*joints_vel)[msg_state->name[i]] = msg_state->velocity[i];
        }
        if ((*joints_pos).size() != 10) {
            return;
        }

        cal_pub_odom(joints_pos, joints_vel);

    }

    void cal_pub_odom(const std::unique_ptr<std::unordered_map<std::string, double>>& joints_pos,
                   const std::unique_ptr<std::unordered_map<std::string, double>>& joints_vel)
    {
        //using code from rover.py
        rclcpp::Time t_curr = this->get_clock()->now();
        int32_t t_prev = odom_msg_.header.stamp.sec * 1e9 + odom_msg_.header.stamp.nanosec;
        float dt = (t_curr.nanoseconds() - t_prev) / 1e9;
        forward_kinematics(joints_pos, joints_vel);
        double dx = curr_twist_.twist.linear.x * dt;
        double dtheta = curr_twist_.twist.angular.z * dt;
        double current_angle = 2 * atan2(odom_msg_.pose.pose.orientation.z, odom_msg_.pose.pose.orientation.w);
        double new_angle = current_angle + dtheta;
        odom_msg_.pose.pose.orientation.z = sin(new_angle / 2);
        odom_msg_.pose.pose.orientation.w = cos(new_angle / 2);
        odom_msg_.pose.pose.position.x += dx * cos(new_angle);
        odom_msg_.pose.pose.position.y += dx * sin(new_angle);
        std::array<double, 36> covariance_matrix = {0.0};
        covariance_matrix[0] = 0.0225;
        covariance_matrix[5] = 0.01;
        covariance_matrix[36 - 5] = 0.0225;
        covariance_matrix[36 - 1] = 0.04;
        odom_msg_.pose.covariance = covariance_matrix;
        odom_msg_.twist = curr_twist_;
        odom_msg_.twist.covariance = covariance_matrix;
        odom_msg_.header.stamp = t_curr;
        odom_pub_->publish(odom_msg_);


        // static tf2_ros::TransformBroadcaster transform_broadcaster(this);
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = t_curr;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = odom_msg_.pose.pose.position.x;
        transform_stamped.transform.translation.y = odom_msg_.pose.pose.position.y;

        transform_stamped.transform.rotation = odom_msg_.pose.pose.orientation;
        transform_broadcaster_->sendTransform(transform_stamped);
    //    printf("x_pos : %f\ty_pos : %f\n", x_postion,y_postion);
    }

    void forward_kinematics(const std::unique_ptr<std::unordered_map<std::string, double>>& joints_pos,
                   const std::unique_ptr<std::unordered_map<std::string, double>>& joints_vel)
    {
        double r_fl, r_fr, r_ml, r_mr, r_rl, r_rr;
        double vel_ml = joints_vel->at("middle_wheel_joint_left");
        double vel_mr = joints_vel->at("middle_wheel_joint_right");
        double ang_fl = -joints_pos->at("front_wheel_joint_L");
        double ang_fr = -joints_pos->at("front_wheel_joint_R");  
        double ang_rl = -joints_pos->at("rear_wheel_joint_L");
        double ang_rr = -joints_pos->at("rear_wheel_joint_R");

        if (ang_fl + ang_fr + ang_rl + ang_rr > 0) {    //turning left
            r_fl = d1 + angle_to_turning_radius(ang_fl, true);
            r_fr = -d1 + angle_to_turning_radius(ang_fr, true);
            r_rl = -d1 - angle_to_turning_radius(ang_rl, false);
            r_rr = d1 - angle_to_turning_radius(ang_rr, false);
        }
        else  { //turning right
            r_fr = d1 + angle_to_turning_radius(ang_fl, true);
            r_fl = -d1 + angle_to_turning_radius(ang_fr, true);
            r_rr = -d1 - angle_to_turning_radius(ang_rl, false);
            r_rl = d1 - angle_to_turning_radius(ang_rr, false);
        }
        std::vector<double> r_list = {r_fl, r_fr, r_ml, r_mr, r_rl, r_rr};
        std::sort(r_list.begin(), r_list.end());
        double r = (r_list[1] + r_list[2]) / 2.0;
        double angular_velocity_center = (vel_ml + vel_mr) / 2;
        curr_twist_.twist.linear.x = WHEEL_RADIUS * angular_velocity_center;
        if (r == 0) {
            curr_twist_.twist.linear.x = 0;
            curr_twist_.twist.angular.z = angular_velocity_center * WHEEL_RADIUS / d4;
        }
        else {
            curr_twist_.twist.angular.z = curr_twist_.twist.linear.x / r;
        }
    }

    // void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    //     tf2::Quaternion quaternion;
    //     tf2::fromMsg(msg->orientation, quaternion);

    //     // Convert quaternion to roll, pitch, yaw
    //     double roll, pitch, yaw;
    //     tf2::Matrix3x3 m(quaternion);
    //     m.getRPY(roll, pitch, yaw);
    //     theta = yaw;

    // }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}