#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <string>
#include <map>

/**
 * Spiel-Controller Roboter-Steuerungs-Knoten
 * 
 * Dieser Knoten fungiert als Brücke zwischen Spiel-Controller-Eingaben (von Knoten #8)
 * und Roboter-Motorsteuerung (zu Knoten #12). Er übersetzt Joystick-Bewegungen
 * in Roboter-Geschwindigkeitsbefehle mit angemessener Skalierung und Sicherheitsfeatures.
 */
class GameControllerRobotControl 
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Abonnenten und Herausgeber
    ros::Subscriber joy_subscriber_;
    ros::Subscriber controller_status_subscriber_;
    std::vector<ros::Publisher> twist_publishers_;
    
    // Konfigurationsparameter
    int num_robots_;
    int current_robot_index_;
    std::vector<std::string> robot_topics_;
    
    // Joystick-Konfiguration
    int left_stick_x_axis_;    // Linker Joystick X (seitliche Bewegung links/rechts)
    int left_stick_y_axis_;    // Linker Joystick Y (vorwärts/rückwärts) 
    int right_stick_x_axis_;   // Rechter Joystick X (Rotation)
    int robot_select_button_;  // Taste zum Durchschalten zwischen Robotern
    
    // Skalierungsfaktoren für sanfte Steuerung
    double linear_scale_x_;    // Skalierung für seitliche Bewegung
    double linear_scale_y_;    // Skalierung für vorwärts/rückwärts-Bewegung  
    double angular_scale_;     // Skalierung für Rotation
    
    // Sicherheit und Zustandsverwaltung
    bool controller_connected_;
    bool button_pressed_last_cycle_;
    ros::Time last_joy_message_time_;
    double connection_timeout_;
    
    // Not-Stopp-Funktionalität
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
        
        // Setze Anfangs-Motorgeschwindigkeiten auf 0 wie gefordert
        setAllRobotsToZero();
        
        // Starte Sicherheits-Überwachungstimer
        safety_timer_ = nh_.createTimer(ros::Duration(0.1), 
                                       &GameControllerRobotControl::safetyTimerCallback, this);
        
        ROS_INFO("Spiel-Controller Roboter-Steuerungs-Knoten initialisiert mit %d Robotern", num_robots_);
        ROS_INFO("Steuere momentan Roboter %d auf Topic: %s", 
                 current_robot_index_, robot_topics_[current_robot_index_].c_str());
    }
    
private:
    /**
     * Initialisiere alle Konfigurationsparameter vom ROS-Parameter-Server
     * Dies ermöglicht flexible Konfiguration ohne Neukompilierung
     */
    void initializeParameters()
    {
        // Roboter-Konfiguration
        private_nh_.param<int>("num_robots", num_robots_, 1);
        
        // Generiere Standard-Topic-Namen falls nicht angegeben
        std::vector<std::string> default_topics;
        for(int i = 0; i < num_robots_; i++) {
            default_topics.push_back("/robot_" + std::to_string(i) + "/cmd_vel");
        }
        private_nh_.param<std::vector<std::string>>("robot_topics", robot_topics_, default_topics);
        
        // Joystick-Achsen-Zuordnung (Xbox-Controller-Layout als Standard)
        private_nh_.param<int>("left_stick_x_axis", left_stick_x_axis_, 0);    // Linker Stick X
        private_nh_.param<int>("left_stick_y_axis", left_stick_y_axis_, 1);    // Linker Stick Y  
        private_nh_.param<int>("right_stick_x_axis", right_stick_x_axis_, 3);  // Rechter Stick X
        private_nh_.param<int>("robot_select_button", robot_select_button_, 0); // A-Taste
        
        // Bewegungs-Skalierungsfaktoren - abgestimmt für sanfte, responsive Steuerung
        private_nh_.param<double>("linear_scale_x", linear_scale_x_, 1.0);     // m/s max seitliche Geschwindigkeit
        private_nh_.param<double>("linear_scale_y", linear_scale_y_, 1.5);     // m/s max Vorwärtsgeschwindigkeit
        private_nh_.param<double>("angular_scale", angular_scale_, 2.0);       // rad/s max Rotationsgeschwindigkeit
        
        // Sicherheitsparameter
        private_nh_.param<double>("connection_timeout", connection_timeout_, 1.0); // Sekunden
        
        // Validiere Konfiguration
        if(robot_topics_.size() != static_cast<size_t>(num_robots_)) {
            ROS_WARN("Anzahl der Roboter-Topics (%zu) stimmt nicht mit num_robots (%d) überein. Anpassung...", 
                     robot_topics_.size(), num_robots_);
            num_robots_ = robot_topics_.size();
        }
    }
    
    /**
     * Erstelle alle ROS-Kommunikationskanäle
     */
    void setupPublishersAndSubscribers()
    {
        // Abonniere Joy-Nachrichten von Knoten #8
        joy_subscriber_ = nh_.subscribe("/joy", 10, 
                                       &GameControllerRobotControl::joyCallback, this);
        
        // Optional: Abonniere Controller-Verbindungsstatus falls verfügbar
        controller_status_subscriber_ = nh_.subscribe("/joy_node/connection_status", 10,
                                                     &GameControllerRobotControl::connectionStatusCallback, this);
        
        // Erstelle Herausgeber für jeden Roboter-Motorsteuerung (zu Knoten #12)
        twist_publishers_.resize(num_robots_);
        for(int i = 0; i < num_robots_; i++) {
            twist_publishers_[i] = nh_.advertise<geometry_msgs::Twist>(robot_topics_[i], 1);
            ROS_INFO("Herausgeber für Roboter %d auf Topic erstellt: %s", i, robot_topics_[i].c_str());
        }
    }
    
    /**
     * Haupt-Callback-Funktion die Joystick-Eingaben verarbeitet und Roboter-Befehle generiert
     * Hier findet die Kern-Joystick-zu-Roboter-Übersetzung statt
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        last_joy_message_time_ = ros::Time::now();
        controller_connected_ = true;
        
        // Validiere dass Joystick-Nachricht genügend Achsen und Tasten hat
        if(!validateJoyMessage(joy_msg)) {
            ROS_WARN_THROTTLE(1.0, "Ungültige Joystick-Nachricht empfangen");
            return;
        }
        
        // Behandle Roboter-Auswahl (Nice-to-have Feature)
        handleRobotSelection(joy_msg);
        
        // Generiere Bewegungsbefehl aus Joystick-Eingabe
        geometry_msgs::Twist twist_cmd = generateTwistCommand(joy_msg);
        
        // Sende Befehle an Roboter
        sendCommandsToRobots(twist_cmd);
    }
    
    /**
     * Validiere dass die Joystick-Nachricht alle benötigten Achsen und Tasten enthält
     */
    bool validateJoyMessage(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        int required_axes = std::max({left_stick_x_axis_, left_stick_y_axis_, right_stick_x_axis_}) + 1;
        int required_buttons = robot_select_button_ + 1;
        
        if(static_cast<int>(joy_msg->axes.size()) < required_axes) {
            ROS_ERROR_THROTTLE(5.0, "Joy-Nachricht hat %zu Achsen, benötigt mindestens %d", 
                              joy_msg->axes.size(), required_axes);
            return false;
        }
        
        if(static_cast<int>(joy_msg->buttons.size()) < required_buttons) {
            ROS_ERROR_THROTTLE(5.0, "Joy-Nachricht hat %zu Tasten, benötigt mindestens %d", 
                              joy_msg->buttons.size(), required_buttons);
            return false;
        }
        
        return true;
    }
    
    /**
     * Behandle Roboter-Auswahl über Controller-Taste (Nice-to-have Feature)
     * Schaltet durch verfügbare Roboter wenn Taste gedrückt wird
     */
    void handleRobotSelection(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        if(num_robots_ <= 1) return; // Keine Auswahl nötig bei nur einem Roboter
        
        bool button_pressed = joy_msg->buttons[robot_select_button_] > 0;
        
        // Erkenne Tastendruck (steigende Flanke)
        if(button_pressed && !button_pressed_last_cycle_) {
            // Stoppe aktuellen Roboter vor dem Wechsel
            publishTwistToRobot(current_robot_index_, geometry_msgs::Twist());
            
            // Wechsle zum nächsten Roboter
            current_robot_index_ = (current_robot_index_ + 1) % num_robots_;
            
            ROS_INFO("Gewechselt zur Steuerung von Roboter %d auf Topic: %s", 
                     current_robot_index_, robot_topics_[current_robot_index_].c_str());
        }
        
        button_pressed_last_cycle_ = button_pressed;
    }
    
    /**
     * Wandle Joystick-Eingabe in Roboter-Geschwindigkeitsbefehl um
     * Dies implementiert die Kern-Zuordnungsanforderungen:
     * - Linker Stick X → Roboter X-Geschwindigkeit (seitlich)
     * - Linker Stick Y → Roboter Y-Geschwindigkeit (vorwärts/rückwärts) 
     * - Rechter Stick X → Roboter Winkelgeschwindigkeit (Rotation)
     */
    geometry_msgs::Twist generateTwistCommand(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        geometry_msgs::Twist twist;
        
        // Ordne Joystick-Achsen zu Roboter-Geschwindigkeiten mit angemessener Skalierung zu
        // Hinweis: Joystick Y ist typischerweise invertiert (oben = negativ), daher negieren wir
        twist.linear.x = joy_msg->axes[left_stick_x_axis_] * linear_scale_x_;  // Seitlich links/rechts
        twist.linear.y = -joy_msg->axes[left_stick_y_axis_] * linear_scale_y_; // Vorwärts/rückwärts (invertiert)
        twist.linear.z = 0.0; // Keine vertikale Bewegung für Bodenroboter
        
        // Rotation: rechter Stick X steuert Gier-Rotation
        // Positive Stick-Eingabe (rechts) = positive Winkelgeschwindigkeit (gegen Uhrzeigersinn nach ROS-Konvention)
        // Aber die Anforderung besagt rechts = im Uhrzeigersinn, daher negieren wir
        twist.angular.x = 0.0; // Kein Rollen
        twist.angular.y = 0.0; // Kein Nicken  
        twist.angular.z = -joy_msg->axes[right_stick_x_axis_] * angular_scale_; // Gieren (negiert für Uhrzeigersinn)
        
        // Wende Totzone an um winzige Bewegungen durch Stick-Drift zu verhindern
        applyDeadzone(twist);
        
        return twist;
    }
    
    /**
     * Wende Totzone-Filterung an um ungewollte Bewegungen durch Controller-Drift zu verhindern
     * Kleine Joystick-Werte werden auf Null gesetzt um saubere Stopps zu gewährleisten
     */
    void applyDeadzone(geometry_msgs::Twist& twist)
    {
        const double deadzone = 0.1; // 10% Totzone
        
        if(std::abs(twist.linear.x) < deadzone) twist.linear.x = 0.0;
        if(std::abs(twist.linear.y) < deadzone) twist.linear.y = 0.0;
        if(std::abs(twist.angular.z) < deadzone) twist.angular.z = 0.0;
    }
    
    /**
     * Sende Bewegungsbefehle an entsprechende Roboter
     * Aktiver Roboter erhält den Befehl, andere erhalten Null-Geschwindigkeit (Nice-to-have Anforderung)
     */
    void sendCommandsToRobots(const geometry_msgs::Twist& twist_cmd)
    {
        for(int i = 0; i < num_robots_; i++) {
            if(i == current_robot_index_) {
                // Sende tatsächlichen Befehl an aktiven Roboter
                publishTwistToRobot(i, twist_cmd);
            } else {
                // Sende Null-Befehl an inaktive Roboter (Nice-to-have Anforderung)
                publishTwistToRobot(i, geometry_msgs::Twist());
            }
        }
    }
    
    /**
     * Veröffentliche Twist-Befehl an spezifischen Roboter
     */
    void publishTwistToRobot(int robot_index, const geometry_msgs::Twist& twist)
    {
        if(robot_index >= 0 && robot_index < num_robots_) {
            twist_publishers_[robot_index].publish(twist);
        }
    }
    
    /**
     * Setze alle Roboter auf Null-Geschwindigkeit - verwendet für Initialisierung und Not-Stopps
     */
    void setAllRobotsToZero()
    {
        geometry_msgs::Twist zero_twist;
        for(int i = 0; i < num_robots_; i++) {
            publishTwistToRobot(i, zero_twist);
        }
        ROS_INFO("Alle Roboter auf Null-Geschwindigkeit gesetzt");
    }
    
    /**
     * Behandle Controller-Verbindungsstatus-Updates (falls verfügbar von Knoten #8)
     */
    void connectionStatusCallback(const std_msgs::Bool::ConstPtr& status_msg)
    {
        bool was_connected = controller_connected_;
        controller_connected_ = status_msg->data;
        
        if(was_connected && !controller_connected_) {
            ROS_WARN("Spiel-Controller getrennt - stoppe alle Roboter");
            setAllRobotsToZero();
        } else if(!was_connected && controller_connected_) {
            ROS_INFO("Spiel-Controller verbunden");
        }
    }
    
    /**
     * Sicherheitstimer-Callback - überwacht Controller-Zeitüberschreitungen
     * Falls keine Joystick-Nachrichten innerhalb der Zeitüberschreitung empfangen, nehme Trennung an
     */
    void safetyTimerCallback(const ros::TimerEvent& event)
    {
        if(controller_connected_) {
            double time_since_last_message = (ros::Time::now() - last_joy_message_time_).toSec();
            
            if(time_since_last_message > connection_timeout_) {
                ROS_WARN("Keine Joystick-Nachrichten seit %.1f Sekunden - nehme Controller-Trennung an", 
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