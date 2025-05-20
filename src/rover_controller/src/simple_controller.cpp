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


////////////////////////////////////////////////////////////////////////////////////////
#include <chrono>
#include <memory>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"


#include "sensor_msgs/msg/imu.hpp"


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
// #include "tf2_ros/transform_broadcaster.h"

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

    WheelServoCommand& operator=(const WheelServoCommand& other) {
        if (this != &other) {
            vel_fl = other.vel_fl;
            vel_fr = other.vel_fr;
            vel_ml = other.vel_ml;
            vel_mr = other.vel_mr;
            vel_rl = other.vel_rl;
            vel_rr = other.vel_rr;
            angle_fl = other.angle_fl;
            angle_fr = other.angle_fr;
            angle_rl = other.angle_rl;
            angle_rr = other.angle_rr;
        }
        return *this;
    }
} WheelServoCommand_;

class SimpleController : public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        

        // Odometry
        // double wheel_radius_;
        // double wheel_separation_;
        // Eigen::Matrix2d speed_conversion_;
        // double right_wheel_prev_pos_;
        // double left_wheel_prev_pos_;
        // double x_;
        // double y_;
        // double theta_;

        rclcpp::Time prev_time_;

        // TF
        

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_cmd_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

        double theta = 0;
        // double l = 0; // linear, angular turning radius

        // double theta_front_closest;
        // double theta_front_farthest;

        // double angular_velocity_center, vel_middle_closest, vel_corner_closest, vel_corner_farthest, vel_middle_farthest;
        // double ang_vel_middle_closest, ang_vel_corner_closest, ang_vel_corner_farthest, ang_vel_middle_farthest;
        double start_time, time, pre_time;

        geometry_msgs::msg::Twist pri_velocity;


        double fl_vel, fr_vel, ml_vel, mr_vel, rl_vel, rr_vel;
        double current_dl, dl, pre_dl;
        double x_postion, y_postion;

        bool delay_ = true;
        nav_msgs::msg::Odometry odom_msg_;
        std::unordered_map<std::string, double> joints_pos_ = {};
        std::unordered_map<std::string, double> joints_vel_ = {};
        geometry_msgs::msg::TwistWithCovariance curr_twist_ = geometry_msgs::msg::TwistWithCovariance();

public:
    SimpleController(const std::string& name) : Node(name) {
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/rover_controller/cmd_vel", 1, std::bind(&SimpleController::cmd_velCallback, this, _1));
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&SimpleController::jointStateCallback, this, _1));
        wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 1);
        servo_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/servo_controller/joint_trajectory", 1);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rover_controller/odom", 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/out", 1, std::bind(&SimpleController::imuCallback, this, std::placeholders::_1));

        transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_msg_.header.stamp = this->get_clock()->now();
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_footprint";
        odom_msg_.pose.pose.orientation.w = 1;

        prev_time_ = get_clock()->now();
    }

    //whenever a new velocity command (message) is received this callback function is called
    //joystick publishes the velocity command in the form of a TwistStamped message
    //This callback function converts the velocity command to wheel speeds
    //and publishes to the wheel controller
    //the wheel controller is a separate node that takes care of the low level control of the wheels
    void cmd_velCallback(const geometry_msgs::msg::TwistStamped &msg_twist) {
        if((pri_velocity.linear.x == msg_twist.twist.linear.x) && (pri_velocity.angular.z == msg_twist.twist.angular.z)) return;
        WheelServoCommand_ command;

        if(delay_)
        {
            delay_ = false;

            if (msg_twist.twist.angular.z == 0 && msg_twist.twist.linear.x != 0) { // linear velocity
                command = go_straight(msg_twist);
                publishCommand(command);
                RCLCPP_INFO(this->get_logger(), "going straight");
            }

            else if((msg_twist.twist.angular.z != 0)&&(msg_twist.twist.linear.x == 0))  // rotate in place
            {
                // servo_cmd.angle_fl = -atan(d3/d1);
                // servo_cmd.angle_fr = atan(d3/d1);
                // servo_cmd.angle_rl = atan(d2/d1);
                // servo_cmd.angle_rr = -atan(d2/d1);

                command = rotate_in_place(msg_twist);
                publishCommand(command);
                RCLCPP_INFO(this->get_logger(), "rotate in place");
            }

            else if((msg_twist.twist.angular.z != 0)&&(msg_twist.twist.linear.x != 0)) // rotation
            {
                // 1. Calculate turning radius
                double r_desired = twist_to_turning_radius(msg_twist);

                // 2. Calculate servo motor angle

                // 3. Calculate wheel angular velocity
                command = calculate_control_command(msg_twist.twist.linear.x, r_desired);

                // 4. publish
                publishCommand(command);
                RCLCPP_INFO(this->get_logger(), "fl:%f, fr:%f, ml:%f, mr:%f, rl:%f, rr:%f", command.vel_fl, command.vel_fr, command.vel_ml, command.vel_mr, command.vel_rl, command.vel_rr);
            }

            else if((msg_twist.twist.angular.z == 0)&&(msg_twist.twist.linear.x == 0)) {
                command = stop();
                publishCommand(command);
                RCLCPP_INFO(this->get_logger(), "stop");

            }

            delay_ = true;
        }

        pri_velocity.linear.x = msg_twist.twist.linear.x;
        pri_velocity.angular.z = msg_twist.twist.angular.z;
    }


    WheelServoCommand_ calculate_control_command(float velocity, double r) {
        WheelServoCommand_ command;

        /* * Calculate the required angles of the 4 servo(corner) motors based on the turning radius
        A positive servo angle means turning right, a negative servo angle means turning left
        */
        double theta_fl = atan2(d3, abs(r) - d1);
        double theta_fr = atan2(d3, abs(r) + d1);
        double theta_rl = atan2(d2, abs(r) - d1);
        double theta_rr = atan2(d2, abs(r) + d1);

        if(r > 0) {
            command.angle_fl = -theta_fl;
            command.angle_fr = -theta_fr;
            command.angle_rl = theta_rl;
            command.angle_rr = theta_rr;
        }
        else {
            command.angle_fl = theta_fr;
            command.angle_fr = theta_fl;
            command.angle_rl = -theta_rr;
            command.angle_rr = -theta_rl;
        }
        if (velocity == 0) {
            command.vel_fl = 0.0;
            command.vel_rl = 0.0;

            command.vel_ml = 0.0;
            command.vel_fr = 0.0;

            command.vel_rr = 0.0;
            command.vel_mr = 0.0;
            return command;
        }
        double angular_velocity_center = velocity / abs(r);

        if (abs(r) > r_MAX) {
            command.vel_fl = float(angular_velocity_center);
            command.vel_rl = float(angular_velocity_center);

            command.vel_ml = float(angular_velocity_center);
            command.vel_fr = float(angular_velocity_center);

            command.vel_rr = float(angular_velocity_center);
            command.vel_mr = float(angular_velocity_center);
        }
        else {
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
                command.vel_fl = float(ang_vel_fl);
                command.vel_fr = float(ang_vel_fr);
                command.vel_ml = float(ang_vel_ml);
                command.vel_mr = float(ang_vel_mr);
                command.vel_rl = float(ang_vel_rl);
                command.vel_rr = float(ang_vel_rr);
            }
            else        // turning right
            {
                command.vel_fl = float(ang_vel_fr);
                command.vel_fr = float(ang_vel_fl);
                command.vel_mr = float(ang_vel_ml);
                command.vel_ml = float(ang_vel_mr);
                command.vel_rl = float(ang_vel_rr);
                command.vel_rr = float(ang_vel_rl);

            }
        }
        return command;
    }

    static WheelServoCommand_ stop() {
        WheelServoCommand_ command;
        command.vel_fl = 0;
        command.vel_fr = 0;
        command.vel_ml = 0;
        command.vel_mr = 0;
        command.vel_rl = 0;
        command.vel_rr = 0;
        command.angle_fl = 0;
        command.angle_fr = 0;
        command.angle_rl = 0;
        command.angle_rr = 0;
        return command;
    }

    static WheelServoCommand_ go_straight(const geometry_msgs::msg::TwistStamped &msg) {
        WheelServoCommand_ command;
        double linear_vel = msg.twist.linear.x / WHEEL_RADIUS;
        command.vel_fl = linear_vel;
        command.vel_fr = linear_vel;
        command.vel_ml = linear_vel;
        command.vel_mr = linear_vel;
        command.vel_rl = linear_vel;
        command.vel_rr = linear_vel;
        command.angle_fl = 0;
        command.angle_fr = 0;
        command.angle_rl = 0;
        command.angle_rr = 0;
        return command;
    }

    static double twist_to_turning_radius(const geometry_msgs::msg::TwistStamped &msg) {
        double infinity = std::numeric_limits<double>::infinity();
        if (msg.twist.angular.z == 0) {
            
            return infinity; // No turning radius if angular velocity is zero
        }
        double turning_radius = msg.twist.linear.x / msg.twist.angular.z;
        if (turning_radius == 0) {
            return r_MAX; 
        } 
        if (turning_radius > 0) {
            turning_radius = std::min(r_MIN, std::min(r_MAX, turning_radius));
        }
        else {
            turning_radius = std::max(-r_MAX, std::min(-r_MIN, turning_radius)); 
        }
        return turning_radius;
    }

    static double angle_to_turning_radius(double angle, bool is_front) {
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
        double turning_radius = distance / angle;
        return turning_radius;
    }
    
    static WheelServoCommand_ rotate_in_place(const geometry_msgs::msg::TwistStamped &msg) {
        WheelServoCommand_ wheel_servo_cmd;
        wheel_servo_cmd.vel_fl = -float(std::hypot(d1, d3) * msg.twist.angular.z / WHEEL_RADIUS);
        wheel_servo_cmd.vel_rl = -float(std::hypot(d1, d2) * msg.twist.angular.z / WHEEL_RADIUS);

        wheel_servo_cmd.vel_ml = -float(d4 * msg.twist.angular.z / WHEEL_RADIUS);
        wheel_servo_cmd.vel_fr = float(std::hypot(d1, d3) * msg.twist.angular.z / WHEEL_RADIUS);

        wheel_servo_cmd.vel_rr = float(d4 * msg.twist.angular.z / WHEEL_RADIUS);
        wheel_servo_cmd.vel_mr = float(std::hypot(d1, d2) * msg.twist.angular.z / WHEEL_RADIUS);
        return wheel_servo_cmd;
    }

    void publishCommand(const WheelServoCommand_& wheel_servo_cmd) {
        auto wheel_msg = std_msgs::msg::Float64MultiArray();
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

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg_state) {
        std::unordered_map<std::string, double> joint_states = {};
        for (size_t i = 0; i < msg_state->name.size(); ++i) {
            joint_states[msg_state->name[i]] = msg_state->position[i];
        }
        fl_vel = joint_states["front_wheel_joint_left"];
        fr_vel = joint_states["front_wheel_joint_right"];
        ml_vel = joint_states["middle_wheel_joint_left"];
        mr_vel = joint_states["middle_wheel_joint_right"];
        rl_vel = joint_states["rear_wheel_joint_left"];
        rr_vel = joint_states["rear_wheel_joint_right"];

        Odometry(theta);
    }

    void jointStateCallback2(const sensor_msgs::msg::JointState::SharedPtr msg_state) {
        for (size_t i = 0; i < msg_state->name.size(); ++i) {
            joints_pos_[msg_state->name[i]] = msg_state->position[i];
        }
        for (size_t i = 0; i < msg_state->name.size(); ++i) {
            joints_vel_[msg_state->name[i]] = msg_state->velocity[i];
        }

        Odometry2(&joints_pos_, &joints_vel_);
    }

    void Odometry2(std::unordered_map<std::string, double> *joints_pos,
                   std::unordered_map<std::string, double> *joints_vel) {
        //using code from rover.py
        int32_t t_curr = this->now().nanoseconds();
        int32_t t_prev = odom_msg_.header.stamp.sec * 1e9 + odom_msg_.header.stamp.nanosec;
        float dt = (t_curr - t_prev) / 1e9;
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
        odom_msg_.pose.covariance = covariance_matrix;
        odom_msg_.twist = curr_twist_;
        odom_msg_.twist.covariance = covariance_matrix;
        odom_msg_.header.stamp = this->get_clock()->now();


        static tf2_ros::TransformBroadcaster transform_broadcaster(this);
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = odom_msg_.pose.pose.position.x;
        transform_stamped.transform.translation.y = odom_msg_.pose.pose.position.y;

        transform_stamped.transform.rotation = odom_msg_.pose.pose.orientation;
        transform_broadcaster.sendTransform(transform_stamped);

        odom_pub_->publish(odom_msg_);

    //    printf("x_pos : %f\ty_pos : %f\n", x_postion,y_postion);
    }

    void forward_kinematics(std::unordered_map<std::string, double> *joints_pos,
        std::unordered_map<std::string, double> *joints_vel) {
        double r_fl, r_fr, r_ml, r_mr, r_rl, r_rr;
        double vel_ml = joints_vel->at("middle_wheel_joint_left");
        double vel_mr = joints_vel->at("middle_wheel_joint_right");
        double ang_fl = joints_pos->at("front_wheel_joint_L");
        double ang_fr = joints_pos->at("front_wheel_joint_R");  
        double ang_rl = joints_pos->at("rear_wheel_joint_L");
        double ang_rr = joints_pos->at("rear_wheel_joint_R");

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
        curr_twist_.twist.angular.z = curr_twist_.twist.linear.x / r;

    }

    void Odometry(double angle) {
        //cheated a little by getting theta(angle) from the IMU instead of calculating it from turning radius and servo angles


        current_dl = (fl_vel + fr_vel + ml_vel + mr_vel + rl_vel + rr_vel) * WHEEL_RADIUS / 6;
        dl = current_dl - pre_dl;

        pre_dl = current_dl;

        x_postion += dl * cos(angle);
        y_postion += dl * sin(angle);
        // RCLCPP_INFO(this->get_logger(), "x_position: %f, y_position: %f", x_postion, y_postion);
        // RCLCPP_INFO(this->get_logger(), "****angle: %f", angle);
        odom_msg_.header.stamp = this->get_clock()->now();
        odom_msg_.header.frame_id = "odom";

        odom_msg_.pose.pose.position.x = x_postion;
        odom_msg_.pose.pose.position.y = y_postion;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, angle);

        odom_msg_.pose.pose.orientation.x = quaternion.x();
        odom_msg_.pose.pose.orientation.y = quaternion.y();
        odom_msg_.pose.pose.orientation.z = quaternion.z();
        odom_msg_.pose.pose.orientation.w = quaternion.w();

        static tf2_ros::TransformBroadcaster transform_broadcaster(this);
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = x_postion;
        transform_stamped.transform.translation.y = y_postion;
        transform_stamped.transform.translation.z = 0.0;

        transform_stamped.transform.rotation = odom_msg_.pose.pose.orientation;
        transform_broadcaster.sendTransform(transform_stamped);

        odom_pub_->publish(odom_msg_);

    //    printf("x_pos : %f\ty_pos : %f\n", x_postion,y_postion);
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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleController>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}