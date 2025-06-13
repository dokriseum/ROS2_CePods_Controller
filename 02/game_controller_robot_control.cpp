#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <string>
#include <map>

/**
 * Spiel-Controller-Roboter-Kontrollknoten
 *
 * Dieser Knoten fungiert als Brücke zwischen der Eingabe der Spielsteuerung (von Knoten #8)
 * und der Motorsteuerung des Roboters (zu Knoten #12). Er übersetzt Joystick-Bewegungen
 * in Geschwindigkeitsbefehle für den Roboter mit entsprechender Skalierung und Sicherheitsfunktionen.
 */
 
class GameControllerRobotControl 
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Subscribers & Publishers
    ros::Subscriber joy_subscriber_;
    ros::Subscriber controller_status_subscriber_;
    std::vector<ros::Publisher> twist_publishers_;
    
    // Konfigurationsparameter
    int num_robots_;
    int current_robot_index_;
    std::vector<std::string> robot_topics_;
    
    // Joystick-Konfiguration
    int left_stick_x_axis_;    // Linker Joystick X (Schräglauf nach links/rechts)
    int left_stick_y_axis_;    // Linker Joystick Y (vorwärts/rückwärts) 
    int right_stick_x_axis_;   // Rechter Joystick X (Drehung)
    int robot_select_button_;  // Schaltfläche zum Durchlaufen der Roboter
    
    // Skalierungsfaktoren für eine gleichmäßige Steuerung
    double linear_scale_x_;    // Skalierung für seitliche Bewegung
    double linear_scale_y_;    // Skalierung für Vorwärts-/Rückwärtsbewegung
    double angular_scale_;     // Skalierung für Rotation
        
    // Sicherheit und staatliche Verwaltung
    bool controller_connected_;
    bool button_pressed_last_cycle_;
    ros::Time last_joy_message_time_;
    double connection_timeout_;
    
    // Not-Aus-Funktionalität
    ros::Timer safety_timer_;
    
public:
    GameControllerRobotControl() : 
        private_nh_("~"),
        current_robot_index_(0),
        controller_connected_(false),
        button_pressed_last_cycle_(false)
    {
        initializeParameters();
        setupPublishersAndSubscribers();
        
        // Stellen Sie die anfänglichen Motordrehzahlen nach Bedarf auf 0 ein.
        setAllRobotsToZero();
        
        // Timer für Sicherheitsüberwachung starten
        safety_timer_ = nh_.createTimer(ros::Duration(0.1), 
                                       &GameControllerRobotControl::safetyTimerCallback, this);
        
        ROS_INFO("Game Controller Robot Control Node initialized with %d robots", num_robots_);
        ROS_INFO("Currently controlling robot %d on topic: %s", 
                 current_robot_index_, robot_topics_[current_robot_index_].c_str());
    }
    
private:
    /**
     * Initialisierung aller Konfigurationsparameter vom ROS-Parameter-Server
     * Dies ermöglicht eine flexible Konfiguration ohne Neukompilierung
     */
    void initializeParameters()
    {
        // Roboterkonfiguration
        private_nh_.param<int>("num_robots", num_robots_, 1);
        
        // Standardthemennamen generieren, wenn nicht angegeben
        std::vector<std::string> default_topics;
        for(int i = 0; i < num_robots_; i++) {
            default_topics.push_back("/robot_" + std::to_string(i) + "/cmd_vel");
        }
        private_nh_.param<std::vector<std::string>>("robot_topics", robot_topics_, default_topics);
        
        // Joystick-Achsenbelegung (Xbox-Controller-Layout als Standard)
        private_nh_.param<int>("left_stick_x_axis", left_stick_x_axis_, 0);    // Linker Stick X
        private_nh_.param<int>("left_stick_y_axis", left_stick_y_axis_, 1);    // Linker Stick Y
        private_nh_.param<int>("right_stick_x_axis", right_stick_x_axis_, 3);  // Rechter Stick X
        private_nh_.param<int>("robot_select_button", robot_select_button_, 0); // A-Taste
        
        // Skalierungsfaktoren für Bewegungen - abgestimmt auf eine reibungslose, reaktionsschnelle Steuerung
        private_nh_.param<double>("linear_scale_x", linear_scale_x_, 1.0);     // m/s max. seitliche Geschwindigkeit
        private_nh_.param<double>("linear_scale_y", linear_scale_y_, 1.5);     // m/s max. Fahrgeschwindigkeit
        private_nh_.param<double>("angular_scale", angular_scale_, 2.0);       // rad/s maximale Drehgeschwindigkeit
        
        // Sicherheitsparameter
        private_nh_.param<double>("connection_timeout", connection_timeout_, 1.0); // seconds
        
        // Konfiguration validieren
        if(robot_topics_.size() != static_cast<size_t>(num_robots_)) {
            ROS_WARN("Number of robot topics (%zu) doesn't match num_robots (%d). Adjusting...", 
                     robot_topics_.size(), num_robots_);
            num_robots_ = robot_topics_.size();
        }
    }
    
    /**
     * alle ROS-Kommunikationskanäle einrichten
     */
    void setupPublishersAndSubscribers()
    {
        // Subscribe to joy messages from node #8
        joy_subscriber_ = nh_.subscribe("/joy", 10, 
                                       &GameControllerRobotControl::joyCallback, this);
        
        // Optional: Subscribe to controller connection status if available
        controller_status_subscriber_ = nh_.subscribe("/joy_node/connection_status", 10, &GameControllerRobotControl::connectionStatusCallback, this);
        
        // Create publishers for each robot's motor control (to node #12)
        twist_publishers_.resize(num_robots_);
        for(int i = 0; i < num_robots_; i++) {
            twist_publishers_[i] = nh_.advertise<geometry_msgs::Twist>(robot_topics_[i], 1);
            ROS_INFO("Created publisher for robot %d on topic: %s", i, robot_topics_[i].c_str());
        }
    }
    
    /**
     * Main callback function that processes joystick input and generates robot commands
     * This is where the core joystick-to-robot translation happens
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        last_joy_message_time_ = ros::Time::now();
        controller_connected_ = true;
        
        // Validate joystick message has enough axes and buttons
        if(!validateJoyMessage(joy_msg)) {
            ROS_WARN_THROTTLE(1.0, "Invalid joystick message received");
            return;
        }
        
        // Handle robot selection (nice-to-have feature)
        handleRobotSelection(joy_msg);
        
        // Generate movement command from joystick input
        geometry_msgs::Twist twist_cmd = generateTwistCommand(joy_msg);
        
        // Send commands to robots
        sendCommandsToRobots(twist_cmd);
    }
    
    /**
     * Validate that the joystick message contains all required axes and buttons
     */
    bool validateJoyMessage(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        int required_axes = std::max({left_stick_x_axis_, left_stick_y_axis_, right_stick_x_axis_}) + 1;
        int required_buttons = robot_select_button_ + 1;
        
        if(static_cast<int>(joy_msg->axes.size()) < required_axes) {
            ROS_ERROR_THROTTLE(5.0, "Joy message has %zu axes, need at least %d", 
                              joy_msg->axes.size(), required_axes);
            return false;
        }
        
        if(static_cast<int>(joy_msg->buttons.size()) < required_buttons) {
            ROS_ERROR_THROTTLE(5.0, "Joy message has %zu buttons, need at least %d", 
                              joy_msg->buttons.size(), required_buttons);
            return false;
        }
        
        return true;
    }
    
    /**
     * Handle robot selection via controller button (nice-to-have feature)
     * Cycles through available robots when button is pressed
     */
    void handleRobotSelection(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        if(num_robots_ <= 1) return; // No need to select if only one robot
        
        bool button_pressed = joy_msg->buttons[robot_select_button_] > 0;
        
        // Detect button press (rising edge)
        if(button_pressed && !button_pressed_last_cycle_) {
            // Stop current robot before switching
            publishTwistToRobot(current_robot_index_, geometry_msgs::Twist());
            
            // Cycle to next robot
            current_robot_index_ = (current_robot_index_ + 1) % num_robots_;
            
            ROS_INFO("Switched to controlling robot %d on topic: %s", 
                     current_robot_index_, robot_topics_[current_robot_index_].c_str());
        }
        
        button_pressed_last_cycle_ = button_pressed;
    }
    
    /**
     * Convert joystick input to robot velocity command
     * This implements the core mapping requirements:
     * - Left stick X → robot X velocity (strafe)
     * - Left stick Y → robot Y velocity (forward/back) 
     * - Right stick X → robot angular velocity (rotation)
     */
    geometry_msgs::Twist generateTwistCommand(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        geometry_msgs::Twist twist;
        
        // Map joystick axes to robot velocities with proper scaling
        // Note: Joystick Y is typically inverted (up = negative), so we negate it
        twist.linear.x = joy_msg->axes[left_stick_x_axis_] * linear_scale_x_;  // Strafe left/right
        twist.linear.y = -joy_msg->axes[left_stick_y_axis_] * linear_scale_y_; // Forward/backward (inverted)
        twist.linear.z = 0.0; // No vertical movement for ground robots
        
        // Rotation: right stick X controls yaw rotation
        // Positive stick input (right) = positive angular velocity (counterclockwise by ROS convention)
        // But the requirement says right = clockwise, so we negate
        twist.angular.x = 0.0; // No roll
        twist.angular.y = 0.0; // No pitch  
        twist.angular.z = -joy_msg->axes[right_stick_x_axis_] * angular_scale_; // Yaw (negated for clockwise)
        
        // Apply deadzone to prevent tiny movements from stick drift
        applyDeadzone(twist);
        
        return twist;
    }
    
    /**
     * Apply deadzone filtering to prevent unwanted movement from controller drift
     * Small joystick values are set to zero to ensure clean stops
     */
    void applyDeadzone(geometry_msgs::Twist& twist)
    {
        const double deadzone = 0.1; // 10% deadzone
        
        if(std::abs(twist.linear.x) < deadzone) twist.linear.x = 0.0;
        if(std::abs(twist.linear.y) < deadzone) twist.linear.y = 0.0;
        if(std::abs(twist.angular.z) < deadzone) twist.angular.z = 0.0;
    }
    
    /**
     * Send movement commands to appropriate robots
     * Active robot gets the command, others get zero velocity (nice-to-have requirement)
     */
    void sendCommandsToRobots(const geometry_msgs::Twist& twist_cmd)
    {
        for(int i = 0; i < num_robots_; i++) {
            if(i == current_robot_index_) {
                // Send actual command to active robot
                publishTwistToRobot(i, twist_cmd);
            } else {
                // Send zero command to inactive robots (nice-to-have requirement)
                publishTwistToRobot(i, geometry_msgs::Twist());
            }
        }
    }
    
    /**
     * Publish twist command to specific robot
     */
    void publishTwistToRobot(int robot_index, const geometry_msgs::Twist& twist)
    {
        if(robot_index >= 0 && robot_index < num_robots_) {
            twist_publishers_[robot_index].publish(twist);
        }
    }
    
    /**
     * Set all robots to zero velocity - used for initialization and emergency stops
     */
    void setAllRobotsToZero()
    {
        geometry_msgs::Twist zero_twist;
        for(int i = 0; i < num_robots_; i++) {
            publishTwistToRobot(i, zero_twist);
        }
        ROS_INFO("Set all robots to zero velocity");
    }
    
    /**
     * Handle controller connection status updates (if available from node #8)
     */
    void connectionStatusCallback(const std_msgs::Bool::ConstPtr& status_msg)
    {
        bool was_connected = controller_connected_;
        controller_connected_ = status_msg->data;
        
        if(was_connected && !controller_connected_) {
            ROS_WARN("Game controller disconnected - stopping all robots");
            setAllRobotsToZero();
        } else if(!was_connected && controller_connected_) {
            ROS_INFO("Game controller connected");
        }
    }
    
    /**
     * Safety timer callback - monitors for controller timeouts
     * If no joystick messages received within timeout, assume disconnection
     */
    void safetyTimerCallback(const ros::TimerEvent& event)
    {
        if(controller_connected_) {
            double time_since_last_message = (ros::Time::now() - last_joy_message_time_).toSec();
            
            if(time_since_last_message > connection_timeout_) {
                ROS_WARN("No joystick messages for %.1f seconds - assuming controller disconnected", 
                         time_since_last_message);
                controller_connected_ = false;
                setAllRobotsToZero();
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "game_controller_robot_control");
    
    try {
        GameControllerRobotControl controller;
        
        ROS_INFO("Game Controller Robot Control Node started successfully");
        ROS_INFO("Waiting for joystick input...");
        
        ros::spin();
    }
    catch(const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    return 0;
}