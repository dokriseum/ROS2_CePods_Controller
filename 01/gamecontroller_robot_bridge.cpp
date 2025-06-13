/**
 * @file gamecontroller_robot_bridge.cpp
 * @brief Gamecontroller zu Roboter Bridge Node - Implementation
 * 
 * Diese Datei enthält die komplette Implementation der Bridge-Funktionalität.
 * In C++ trennen wir Deklaration (Header) und Implementation für bessere
 * Compile-Zeiten und klarere Code-Organisation.
 * 
 * @author Senior-Entwickler  
 * @version 1.0
 */

#include "gamecontroller_robot_bridge.h"
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace gamecontroller_bridge {

// ===== GamecontrollerRobotBridge Implementation =====

GamecontrollerRobotBridge::GamecontrollerRobotBridge(ros::NodeHandle& node_handle, 
                                                   const BridgeConfig& config)
    : nh_(node_handle)
    , config_(config)
    , last_joy_msg_time_(ros::Time::now())
{
    // Initialisiere Button-Zustandsspeicher
    // In C++ müssen wir explizit die Größe setzen
    previous_button_states_.resize(10, false);  // Annahme: Max 10 Buttons
    
    ROS_INFO("Initialisiere Gamecontroller Robot Bridge...");
    ROS_INFO("Konfiguration: %d Roboter, Deadzone: %.2f, Linear Scale: %.2f, Angular Scale: %.2f",
             config_.num_robots, config_.deadzone, 
             config_.linear_scale_factor, config_.angular_scale_factor);
    
    // Setup aller Publisher für die Roboter
    // Diese Methode erstellt die Map von Roboter-IDs zu Publishern
    setupRobotPublishers();
    
    // Subscriber für Gamecontroller-Input einrichten
    // In C++ verwenden wir boost::bind für Callback-Binding
    joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>(
        config_.gamecontroller_topic, 
        1,  // Queue size - klein halten für Echtzeitverhalten
        boost::bind(&GamecontrollerRobotBridge::gamecontrollerCallback, this, _1)
    );
    
    ROS_INFO("Subscriber erstellt für Topic: %s", config_.gamecontroller_topic.c_str());
}

GamecontrollerRobotBridge::~GamecontrollerRobotBridge() {
    ROS_INFO("Gamecontroller Robot Bridge wird beendet...");
    stop();
}

bool GamecontrollerRobotBridge::start() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (is_active_.load()) {
        ROS_WARN("Bridge ist bereits aktiv");
        return true;
    }
    
    // Alle Roboter initial stoppen
    stopAllRobots();
    
    // Timer für Verbindungsüberwachung starten
    connection_check_timer_ = nh_.createTimer(
        ros::Duration(config_.connection_timeout),
        &GamecontrollerRobotBridge::checkGamepadConnection,
        this
    );
    
    is_active_.store(true);
    ROS_INFO("Gamecontroller Robot Bridge gestartet und bereit");
    return true;
}

void GamecontrollerRobotBridge::stop() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_active_.load()) {
        return;
    }
    
    // Timer stoppen
    connection_check_timer_.stop();
    
    // Alle Roboter stoppen
    stopAllRobots();
    
    is_active_.store(false);
    gamepad_connected_.store(false);
    
    ROS_INFO("Gamecontroller Robot Bridge gestoppt");
}

int GamecontrollerRobotBridge::getCurrentRobotId() const {
    return current_robot_id_.load();
}

bool GamecontrollerRobotBridge::updateConfig(const BridgeConfig& new_config) {
    if (!ConfigManager::validateConfig(new_config)) {
        ROS_ERROR("Ungültige Konfiguration kann nicht angewendet werden");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Prüfe ob sich die Anzahl Roboter geändert hat
    bool robot_count_changed = (config_.num_robots != new_config.num_robots);
    
    config_ = new_config;
    
    // Bei geänderter Roboteranzahl Publisher neu erstellen
    if (robot_count_changed) {
        robot_publishers_.clear();
        setupRobotPublishers();
        
        // Aktuellen Roboter validieren
        if (current_robot_id_.load() >= config_.num_robots) {
            current_robot_id_.store(0);
        }
    }
    
    ROS_INFO("Konfiguration erfolgreich aktualisiert");
    return true;
}

void GamecontrollerRobotBridge::setupRobotPublishers() {
    robot_publishers_.clear();
    
    for (int robot_id = 0; robot_id < config_.num_robots; ++robot_id) {
        // Topic-Name generieren
        std::stringstream topic_name;
        topic_name << config_.robot_control_topic_prefix << "_robot_" << robot_id;
        
        // Publisher erstellen
        // std::make_shared sorgt für sichere Speicherverwaltung
        auto publisher = std::make_shared<ros::Publisher>(
            nh_.advertise<geometry_msgs::Twist>(topic_name.str(), 1)
        );
        
        robot_publishers_[robot_id] = publisher;
        
        ROS_INFO("Publisher für Roboter %d erstellt: %s", robot_id, topic_name.str().c_str());
    }
    
    ROS_INFO("Alle %d Roboter-Publisher erfolgreich erstellt", config_.num_robots);
}

void GamecontrollerRobotBridge::gamecontrollerCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // Diese Methode wird bei jeder Gamecontroller-Nachricht aufgerufen
    // Sie ist das Herzstück der gesamten Steuerungslogik
    
    if (!is_active_.load()) {
        return;  // Bridge ist nicht aktiv
    }
    
    // Thread-sichere Aktualisierung des Verbindungsstatus
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        gamepad_connected_.store(true);
        last_joy_msg_time_ = ros::Time::now();
    }
    
    // Validiere Nachrichteninhalt
    if (!msg) {
        ROS_WARN("Ungültige Gamecontroller-Nachricht empfangen");
        return;
    }
    
    try {
        // Verarbeite Roboter-Auswahl (Buttons)
        handleRobotSelection(msg);
        
        // Verarbeite Bewegungssteuerung (Joysticks)
        handleMovementControl(msg);
        
    } catch (const std::exception& e) {
        ROS_ERROR("Fehler bei Gamecontroller-Verarbeitung: %s", e.what());
        stopAllRobots();  // Sicherheitsmaßnahme bei Fehlern
    }
}

void GamecontrollerRobotBridge::handleRobotSelection(const sensor_msgs::Joy::ConstPtr& msg) {
    // Button-Edge-Detection implementieren
    // Das verhindert mehrfache Triggerung bei gehaltenen Buttons
    
    std::lock_guard<std::mutex> lock(button_state_mutex_);
    
    if (!isValidIndex(msg->buttons, static_cast<size_t>(config_.buttons.previous_robot_button)) ||
        !isValidIndex(msg->buttons, static_cast<size_t>(config_.buttons.next_robot_button))) {
        return;  // Nicht genügend Buttons verfügbar
    }
    
    // Erweitere Button-Array falls nötig
    size_t max_button = std::max(config_.buttons.previous_robot_button, 
                                config_.buttons.next_robot_button);
    if (previous_button_states_.size() <= max_button) {
        previous_button_states_.resize(max_button + 1, false);
    }
    
    // Vorheriger Roboter (L1 Button)
    bool prev_button_current = msg->buttons[config_.buttons.previous_robot_button];
    bool prev_button_previous = previous_button_states_[config_.buttons.previous_robot_button];
    
    if (prev_button_current && !prev_button_previous) {  // Steigende Flanke
        switchRobot(-1);
    }
    previous_button_states_[config_.buttons.previous_robot_button] = prev_button_current;
    
    // Nächster Roboter (R1 Button)
    bool next_button_current = msg->buttons[config_.buttons.next_robot_button];
    bool next_button_previous = previous_button_states_[config_.buttons.next_robot_button];
    
    if (next_button_current && !next_button_previous) {  // Steigende Flanke
        switchRobot(1);
    }
    previous_button_states_[config_.buttons.next_robot_button] = next_button_current;
}

void GamecontrollerRobotBridge::handleMovementControl(const sensor_msgs::Joy::ConstPtr& msg) {
    // Validiere dass genügend Achsen verfügbar sind
    if (!isValidIndex(msg->axes, static_cast<size_t>(config_.axes.left_stick_x)) ||
        !isValidIndex(msg->axes, static_cast<size_t>(config_.axes.left_stick_y)) ||
        !isValidIndex(msg->axes, static_cast<size_t>(config_.axes.right_stick_x))) {
        ROS_WARN_THROTTLE(1.0, "Gamecontroller hat nicht genügend Achsen");
        return;
    }
    
    // Extrahiere rohe Joystick-Werte
    double raw_linear_x = msg->axes[config_.axes.left_stick_x];   // Links/Rechts
    double raw_linear_y = msg->axes[config_.axes.left_stick_y];   // Vor/Zurück
    double raw_angular_z = msg->axes[config_.axes.right_stick_x]; // Rotation
    
    // Anwenden von Deadzone und Skalierung
    // Diese Funktionen sind inline für maximale Performance
    double linear_x = applyDeadzoneAndScaling(raw_linear_x, 
                                            config_.deadzone, 
                                            config_.linear_scale_factor);
    
    double linear_y = applyDeadzoneAndScaling(raw_linear_y, 
                                            config_.deadzone, 
                                            config_.linear_scale_factor);
    
    double angular_z = applyDeadzoneAndScaling(raw_angular_z, 
                                             config_.deadzone, 
                                             config_.angular_scale_factor);
    
    // Invertiere Rotation für intuitive Steuerung
    // Rechter Stick nach rechts = Rotation im Uhrzeigersinn (negatives angular_z)
    angular_z = -angular_z;
    
    // Sende Geschwindigkeit an aktuellen Roboter
    int current_robot = current_robot_id_.load();
    if (!sendVelocityToRobot(current_robot, linear_x, linear_y, angular_z)) {
        ROS_WARN("Fehler beim Senden der Geschwindigkeit an Roboter %d", current_robot);
    }
    
    // Stoppe alle anderen Roboter
    // Das ist kritisch für die Sicherheit - nur ein Roboter soll sich bewegen
    for (const auto& publisher_pair : robot_publishers_) {
        int robot_id = publisher_pair.first;
        if (robot_id != current_robot) {
            sendVelocityToRobot(robot_id, 0.0, 0.0, 0.0);
        }
    }
    
    // Debug-Ausgabe bei Bewegung
    if (config_.debug_velocity_output) {
        if (std::abs(linear_x) > 0.01 || std::abs(linear_y) > 0.01 || std::abs(angular_z) > 0.01) {
            ROS_DEBUG("Roboter %d: vx=%.2f, vy=%.2f, wz=%.2f", 
                     current_robot, linear_x, linear_y, angular_z);
        }
    }
}

inline double GamecontrollerRobotBridge::applyDeadzoneAndScaling(double value, 
                                                               double deadzone, 
                                                               double scale) const {
    // Diese Funktion ist inline für maximale Performance bei häufigen Aufrufen
    
    // Anwenden der Deadzone
    if (std::abs(value) < deadzone) {
        return 0.0;
    }
    
    // Lineare Skalierung außerhalb der Deadzone
    double sign = (value >= 0.0) ? 1.0 : -1.0;
    double scaled_value = (std::abs(value) - deadzone) / (1.0 - deadzone);
    
    return sign * scaled_value * scale;
}

bool GamecontrollerRobotBridge::sendVelocityToRobot(int robot_id, 
                                                  double linear_x, 
                                                  double linear_y, 
                                                  double angular_z) {
    if (!isValidRobotId(robot_id)) {
        ROS_WARN("Ungültige Roboter-ID: %d", robot_id);
        return false;
    }
    
    auto publisher_it = robot_publishers_.find(robot_id);
    if (publisher_it == robot_publishers_.end() || !publisher_it->second) {
        ROS_WARN("Publisher für Roboter %d nicht gefunden", robot_id);
        return false;
    }
    
    // Erstelle Geschwindigkeitsnachricht
    // WICHTIG: geometry_msgs::Twist muss durch den echten Typ von #12 ersetzt werden
    geometry_msgs::Twist velocity_msg;
    velocity_msg.linear.x = linear_x;
    velocity_msg.linear.y = linear_y;
    velocity_msg.linear.z = 0.0;
    velocity_msg.angular.x = 0.0;
    velocity_msg.angular.y = 0.0;
    velocity_msg.angular.z = angular_z;
    
    // Veröffentliche die Nachricht
    try {
        publisher_it->second->publish(velocity_msg);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Fehler beim Veröffentlichen für Roboter %d: %s", robot_id, e.what());
        return false;
    }
}

bool GamecontrollerRobotBridge::switchRobot(int direction) {
    int old_robot = current_robot_id_.load();
    
    // Berechne neue Roboter-ID mit Wraparound
    int new_robot = (old_robot + direction + config_.num_robots) % config_.num_robots;
    
    // Atomare Aktualisierung für Thread-Safety
    current_robot_id_.store(new_robot);
    
    // Stoppe vorherigen Roboter
    sendVelocityToRobot(old_robot, 0.0, 0.0, 0.0);
    
    if (config_.debug_robot_selection) {
        ROS_INFO("Roboter-Auswahl gewechselt von %d zu %d", old_robot, new_robot);
    }
    
    return true;
}

void GamecontrollerRobotBridge::stopAllRobots() {
    for (const auto& publisher_pair : robot_publishers_) {
        int robot_id = publisher_pair.first;
        sendVelocityToRobot(robot_id, 0.0, 0.0, 0.0);
    }
    
    ROS_INFO("Alle %d Roboter gestoppt", static_cast<int>(robot_publishers_.size()));
}

void GamecontrollerRobotBridge::checkGamepadConnection(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    ros::Duration time_since_last_msg = ros::Time::now() - last_joy_msg_time_;
    
    if (time_since_last_msg.toSec() > config_.connection_timeout) {
        if (gamepad_connected_.load()) {
            ROS_WARN("Gamepad-Verbindung verloren! Stoppe alle Roboter.");
            stopAllRobots();
            gamepad_connected_.store(false);
        }
    }
}

// ===== ConfigManager Implementation =====

BridgeConfig ConfigManager::getDefaultConfig() {
    return BridgeConfig{};  // Verwendet die Default-Werte aus der Struktur-Definition
}

BridgeConfig ConfigManager::loadConfig(const std::string& config_file) {
    BridgeConfig config = getDefaultConfig();
    
    if (config_file.empty()) {
        ROS_INFO("Keine Konfigurationsdatei angegeben, verwende Standardwerte");
        return config;
    }
    
    try {
        YAML::Node yaml_config = YAML::LoadFile(config_file);
        loadFromYaml(yaml_config, config);
        
        if (validateConfig(config)) {
            ROS_INFO("Konfiguration erfolgreich geladen aus: %s", config_file.c_str());
        } else {
            ROS_WARN("Geladene Konfiguration ist ungültig, verwende Standardwerte");
            config = getDefaultConfig();
        }
        
    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML-Parsing Fehler in %s: %s", config_file.c_str(), e.what());
        ROS_INFO("Verwende Standardkonfiguration");
    } catch (const std::exception& e) {
        ROS_ERROR("Fehler beim Laden der Konfiguration %s: %s", config_file.c_str(), e.what());
        ROS_INFO("Verwende Standardkonfiguration");
    }
    
    return config;
}

void ConfigManager::loadFromYaml(const YAML::Node& yaml_node, BridgeConfig& config) {
    // Hilfsmakro für sichere YAML-Zugriffe
    #define SAFE_YAML_READ(field, type) \
        if (yaml_node[#field]) { \
            config.field = yaml_node[#field].as<type>(); \
        }
    
    // Lade Basis-Konfiguration
    SAFE_YAML_READ(gamecontroller_topic, std::string);
    SAFE_YAML_READ(robot_control_topic_prefix, std::string);
    SAFE_YAML_READ(num_robots, int);
    SAFE_YAML_READ(deadzone, double);
    SAFE_YAML_READ(linear_scale_factor, double);
    SAFE_YAML_READ(angular_scale_factor, double);
    SAFE_YAML_READ(connection_timeout, double);
    
    // Lade Button-Konfiguration
    if (yaml_node["robot_selection_buttons"]) {
        const auto& buttons = yaml_node["robot_selection_buttons"];
        if (buttons["previous"]) {
            config.buttons.previous_robot_button = buttons["previous"].as<int>();
        }
        if (buttons["next"]) {
            config.buttons.next_robot_button = buttons["next"].as<int>();
        }
    }
    
    // Lade Achsen-Konfiguration
    if (yaml_node["joystick_axes"]) {
        const auto& axes = yaml_node["joystick_axes"];
        if (axes["left_stick_x"]) {
            config.axes.left_stick_x = axes["left_stick_x"].as<int>();
        }
        if (axes["left_stick_y"]) {
            config.axes.left_stick_y = axes["left_stick_y"].as<int>();
        }
        if (axes["right_stick_x"]) {
            config.axes.right_stick_x = axes["right_stick_x"].as<int>();
        }
        if (axes["right_stick_y"]) {
            config.axes.right_stick_y = axes["right_stick_y"].as<int>();
        }
    }
    
    // Lade Debug-Konfiguration
    if (yaml_node["debug"]) {
        const auto& debug = yaml_node["debug"];
        if (debug["log_velocity_commands"]) {
            config.debug_velocity_output = debug["log_velocity_commands"].as<bool>();
        }
        if (debug["show_robot_selection"]) {
            config.debug_robot_selection = debug["show_robot_selection"].as<bool>();
        }
    }
    
    #undef SAFE_YAML_READ
}

bool ConfigManager::validateConfig(const BridgeConfig& config) {
    // Validiere kritische Parameter
    if (config.num_robots <= 0 || config.num_robots > 100) {
        ROS_ERROR("Ungültige Anzahl Roboter: %d (muss zwischen 1 und 100 sein)", config.num_robots);
        return false;
    }
    
    if (config.deadzone < 0.0 || config.deadzone >= 1.0) {
        ROS_ERROR("Ungültige Deadzone: %.2f (muss zwischen 0.0 und 1.0 sein)", config.deadzone);
        return false;
    }
    
    if (config.linear_scale_factor <= 0.0 || config.linear_scale_factor > 10.0) {
        ROS_ERROR("Ungültiger linearer Skalierungsfaktor: %.2f", config.linear_scale_factor);
        return false;
    }
    
    if (config.angular_scale_factor <= 0.0 || config.angular_scale_factor > 20.0) {
        ROS_ERROR("Ungültiger angularer Skalierungsfaktor: %.2f", config.angular_scale_factor);
        return false;
    }
    
    if (config.connection_timeout <= 0.0 || config.connection_timeout > 60.0) {
        ROS_ERROR("Ungültiger Connection-Timeout: %.2f", config.connection_timeout);
        return false;
    }
    
    // Validiere Button-IDs
    if (config.buttons.previous_robot_button < 0 || config.buttons.next_robot_button < 0) {
        ROS_ERROR("Ungültige Button-IDs für Roboter-Auswahl");
        return false;
    }
    
    if (config.buttons.previous_robot_button == config.buttons.next_robot_button) {
        ROS_ERROR("Gleiche Button-IDs für vorherigen und nächsten Roboter");
        return false;
    }
    
    // Validiere Achsen-IDs
    if (config.axes.left_stick_x < 0 || config.axes.left_stick_y < 0 || 
        config.axes.right_stick_x < 0 || config.axes.right_stick_y < 0) {
        ROS_ERROR("Ungültige Achsen-IDs");
        return false;
    }
    
    return true;
}

} // namespace gamecontroller_bridge