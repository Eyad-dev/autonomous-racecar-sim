#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <cmath>

class PidRacer : public rclcpp::Node {
    public:
        PidRacer() : Node("pid_racer"), current_wp_(0){
            //Publishing steering commands
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            // Get the car's position
            subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10, std::bind(&PidRacer::odom_callback, this, std::placeholders::_1));

            waypoints_ = {{25.0, 0.0}, {25.0, 25.0}, {0.0,25.0}, {0.0,0.0}};

            RCLCPP_INFO(this->get_logger(), "PID Racer initialized. Zoom Zoom");
        }

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            //Extract X, Y, and Yaw
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;


            // Converting Quaternion to Euler Yaw
            double qw = msg->pose.pose.orientation.w;
            double qx = msg->pose.pose.orientation.x;
            double qy = msg->pose.pose.orientation.y;
            double qz = msg->pose.pose.orientation.z;
            double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

            //Target place
            double target_x = waypoints_[current_wp_].first;
            double target_y = waypoints_[current_wp_].second;


            // Calcuate the error
            double dx = target_x - x;
            double dy = target_y - y;
            double distance = std::sqrt(dx*dx + dy*dy);
            double target_heading = std::atan2(dy,dx);

            //Normalize Heading error to keep it between -PI and PI
            double heading_error = target_heading - yaw;
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

            //The PID controller
            double steering_kp = 3.5;
            double steering_cmd = steering_kp * heading_error;

            if(steering_cmd >= 1.0) steering_cmd = 1.0;
            if(steering_cmd <= -1.0) steering_cmd = -1.0;
            

            //Dynamic Throttles: Going fast on straights, slow down slightly for tight turns
            double speed_cmd = 25.0 - (std::abs(heading_error)* 8.0);
            if(speed_cmd < 5.0) speed_cmd = 5.0;


            if (distance < 2.0) {
                current_wp_ = (current_wp_ + 1) % waypoints_.size();
                RCLCPP_INFO(this->get_logger(), "Hitting waypoint, turning!");
            }


            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = speed_cmd;
            cmd.angular.z = steering_cmd;
            publisher_->publish(cmd);
        }


        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
        std::vector<std::pair<double, double>> waypoints_;
        size_t current_wp_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidRacer>());
    rclcpp::shutdown();
    return 0;
}