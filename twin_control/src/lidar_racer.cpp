#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class LidarRacer : public rclcpp::Node{
    public:
    LidarRacer() : Node("lidar_racer"){
        //Publisher to steer the wheels
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        //Subscriber to get the lidar readings from python
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarRacer::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "LiDAR Brain Online. Looking for open space...");
    }
    private:
    
        //The 1!=2 error fix I suppose
        bool first_run_ = true;

        //the PID state variables
        rclcpp::Time last_time_;
        double integral_steer_ = 0.0;
        double prev_error_steer = 0.0;


        //PID tuning gains
        //Kp: Proportional
        //Ki: integral
        //Kd: derivative
        double Kp_steer = 1.0;
        double Ki_steer = 0.001;
        double Kd_steer = 0.3;
    
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            //1. Get time delta for PID math
            
            auto current_time = this->now();
            if (first_run_){
                last_time_ = current_time;
                first_run_ = false;
            }
            
            
            double dt = (current_time - last_time_).seconds();
            if (dt <= 0.0) dt = 0.01; //Preventing division by zero in the first run
            last_time_ = current_time;            
            
            int num_of_rays = msg->ranges.size();

            //1.5 Object inflation logic
            std::vector<double> safe_ranges(msg->ranges.begin(), msg->ranges.end());
            double CAR_RADIUS = 0.7; //The width should be 0.5m but added 0.2 as a safety check.


            for (int i = 0; i < num_of_rays; i++){
                if (msg->ranges[i] <10.0){
                    double angle_to_cover = CAR_RADIUS/msg->ranges[i];
                    int rays_to_inflate = std::ceil(angle_to_cover/msg->angle_increment);

                    for (int j = -rays_to_inflate; j<= rays_to_inflate; j++){
                        int neighbor = (i + j + num_of_rays) % num_of_rays;

                        if (msg->ranges[i] < safe_ranges[neighbor]){
                            safe_ranges[neighbor] = msg->ranges[i];
                        }
                    }
                }

            }

            // 2. Find the absolute furthest distance using inflated ranges
            double max_distance = 0.0;
            double best_angle_sum = 0.0;
            int gap_ray_count = 0;
            
            for(int i = 0; i < num_of_rays; i++){
                if (i > 7 && i < 29) continue; // Ignore blind spot
                
                if (safe_ranges[i] > max_distance){
                    max_distance = safe_ranges[i];
                }
            }

            // 3. Average the angles (WITH THE 360-DEGREE MATH FIX)
            for(int i = 0; i < num_of_rays; i++){
                if (i > 7 && i < 29) continue;
                if (safe_ranges[i] >= max_distance - 1.5){
                    double ray_angle = msg->angle_min + (i * msg->angle_increment);
                    
                    // Normalize the angle BEFORE adding it to the sum!
                    // This turns 350 degrees into -10 degrees, so the average works perfectly.
                    while (ray_angle > M_PI) ray_angle -= 2.0 * M_PI;
                    while (ray_angle < -M_PI) ray_angle += 2.0 * M_PI;
                    
                    best_angle_sum += ray_angle;
                    gap_ray_count++;
                }
            }

            // 4. Calculate target angle
            double target_angle = 0.0;
            if (gap_ray_count > 0) {
                target_angle = best_angle_sum / gap_ray_count;
            }

            // The average is already normalized, but do it one more time just in case
            while (target_angle > M_PI) target_angle -= 2.0 * M_PI;
            while (target_angle < -M_PI) target_angle += 2.0 * M_PI;
            

            double front_dist = 15.0;
            int front_rays[] = {34,35,0,1,2};
            for (int i = 0; i < 5; i++){
                if (msg->ranges[front_rays[i]] < front_dist){
                    front_dist = msg->ranges[front_rays[i]];
                }
            }

            double speed_cmd = std::min(10.0, std::max(0.5, front_dist*0.9));

            if (front_dist < 8) {
                target_angle *= 3; 
            }

            double min_left = 10.0;
            double min_right = 10.0;

            // Scan out the LEFT side (Rays 7 to 12)
            // Completely ignores the front windshield so it doesn't panic at red walls
            for (int i = 3; i <= 12; i++) {
                if (msg->ranges[i] < min_left) min_left = msg->ranges[i];
            }
            
            // Scan out the RIGHT side (Rays 24 to 29)
            for (int i = 24; i <= 33; i++) {
                if (msg->ranges[i] < min_right) min_right = msg->ranges[i];
            }
            
            double REPULSION_DIST = 2; // Meters. If the car is closer than this to the wall, it starts applying a force.

            // Override the AI ONLY if the side of the car is scraping the blue apex
            if (min_left < REPULSION_DIST) {
                target_angle -= (REPULSION_DIST - min_left) * 1.5; 
            }
            if (min_right < REPULSION_DIST) {
                target_angle += (REPULSION_DIST - min_right) * 1.5;
            }

            // 5. PID Steering controller
            double error = target_angle; //We want to steer towards the target

            // Proportional
            double P = Kp_steer * error;

            // Integral (with anti-windup to stop runaway math, whatever that is)
            integral_steer_ += error * dt;
            if (integral_steer_ > 1.0) integral_steer_ = 1.0;
            if (integral_steer_ < -1.0) integral_steer_ = -1.0;
            double I = Ki_steer * integral_steer_;

            //Derivative
            double derivative = (error - prev_error_steer) / dt;
            double D = Kd_steer * derivative;

            //Base steering command
            double base_steering = P + I + D; 

            prev_error_steer = error;
            // Calculate weight based on how close the wall is.
            // If front_dist is 10m+, weight is low (car stays straight).
            // If front_dist is 3m, weight is high (car turns aggressively).
            double dist_weight = 1.0 - (std::min(front_dist, 10.0) / 10.0);
            // Map the weight: 0.1 (far) to 1.5 (close)
            double steering_weight = 0.1 + (dist_weight * 1.4);
            double steering_cmd = base_steering * steering_weight;

            

            // Final physical hardware limits
            if (steering_cmd > 0.75) steering_cmd = 0.75;
            if (steering_cmd < -0.75) steering_cmd = -0.75;

            // 6. Proportional speed control

            if (std::abs(steering_cmd) > 0.4 && speed_cmd > 5.0) {
                speed_cmd = 5.0;
            }
            if (front_dist < 1.0) speed_cmd = 0.5;

            // INTELLIGENT DEBUG LOGGER 
            std::string current_state = "🟢 Cruising down the straight";

            if (speed_cmd <= 1.0) {
                current_state = "🛑 EMERGENCY BRAKE! Wall imminent!"; 
            } 
            else if (min_left < REPULSION_DIST || min_right < REPULSION_DIST) {
                // THE NEW EMOJI PRINT: Triggers if the car gets within 1.0m of the side walls
                current_state = "🟡 PANIC DODGE! Scraping the wall!";
            }
            else if (front_dist < 8) {
                current_state = "🟠 Caution: Wall ahead at " + std::to_string(front_dist) + "m, changing direction!";
            }
            else if (std::abs(steering_cmd) >= 0.2) {
                if (steering_cmd > 0) current_state = "🔵 Carving corner: Turning LEFT";
                else current_state = "🔵 Carving corner: Turning RIGHT";
            }

            if (current_state != last_state_) {
                RCLCPP_INFO(this->get_logger(), "%s | Speed: %.1f m/s | Steer: %.2f rad", 
                            current_state.c_str(), speed_cmd, steering_cmd);
                last_state_ = current_state;
            }

            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = speed_cmd;
            cmd.angular.z = steering_cmd;
            publisher_->publish(cmd);
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        std::string last_state_ = "";
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarRacer>());
    rclcpp::shutdown();
    return 0;
}