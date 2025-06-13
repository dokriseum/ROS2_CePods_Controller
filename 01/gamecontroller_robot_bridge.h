/**
 * @file gamecontroller_robot_bridge.h
 * @brief Gamecontroller zu Roboter Bridge Node - Header
 * 
 * Dieser Header definiert die Klasse für die Verbindung zwischen
 * Gamecontroller-Eingaben und Roboter-Motorsteuerung in C++.
 * 
 * @author Senior-Entwickler
 * @version 1.0
 */

#ifndef GAMECONTROLLER_ROBOT_BRIDGE_H
#define GAMECONTROLLER_ROBOT_BRIDGE_H

// ROS Core Includes
#include <ros/ros.h>
#include <ros/console.h>

// Message Type Includes
// WICHTIG: Diese müssen durch die echten Typen von #8 und #12 ersetzt werden
#include <sensor_msgs/Joy.h>          // Gamecontroller Input (Beispiel von #8)
#include <geometry_msgs/Twist.h>      // Roboter Bewegungskommandos (Beispiel für #12)
#include <std_msgs/Int32.h>           // Für Roboter-Auswahl

// Standard Library Includes
#include <map>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <yaml-cpp/yaml.h>

/**
 * @namespace gamecontroller_bridge
 * @brief Namespace für alle Bridge-bezogenen Klassen und Funktionen
 */
namespace gamecontroller_bridge {

/**
 * @struct BridgeConfig
 * @brief Konfigurationsstruktur für den Bridge Node
 * 
 * Diese Struktur kapselt alle konfigurierbaren Parameter.
 * In C++ verwenden wir Strukturen für einfache Datencontainer,
 * was type safety und bessere Performance bietet.
 */
struct BridgeConfig {
    // ROS Topic Konfiguration
    std::string gamecontroller_topic{"/joy"};
    std::string robot_control_topic_prefix{"/robot_cmd_vel"};
    
    // Roboter Konfiguration
    int num_robots{3};
    
    // Steuerungsparameter
    double deadzone{0.1};
    double linear_scale_factor{1.0};
    double angular_scale_factor{2.0};
    
    // Verbindungsüberwachung
    double connection_timeout{1.0};
    
    // Joystick Button/Achsen Mapping
    struct {
        int previous_robot_button{4};  // L1
        int next_robot_button{5};      // R1
    } buttons;
    
    struct {
        int left_stick_x{0};
        int left_stick_y{1};
        int right_stick_x{2};
        int right_stick_y{3};
    } axes;
    
    // Debug Einstellungen
    bool debug_velocity_output{false};
    bool debug_robot_selection{true};
};

/**
 * @class GamecontrollerRobotBridge
 * @brief Hauptklasse für die Gamecontroller-Roboter-Brücke
 * 
 * Diese Klasse implementiert die komplette Logik für die Verbindung
 * zwischen Gamecontroller-Input und Roboter-Steuerung.
 * 
 * Design-Prinzipien:
 * - RAII (Resource Acquisition Is Initialization) für sichere Ressourcenverwaltung
 * - Thread-Safety durch Mutexe für gleichzeitige Zugriffe
 * - Klare Trennung von Verantwortlichkeiten
 * - Fehlerbehandlung auf allen Ebenen
 */
class GamecontrollerRobotBridge {
public:
    /**
     * @brief Konstruktor
     * @param node_handle ROS NodeHandle für diese Instanz
     * @param config Konfigurationsobjekt mit allen Parametern
     * 
     * Der Konstruktor initialisiert alle notwendigen ROS-Publisher und -Subscriber.
     * In C++ verwenden wir Initialisierungslisten für bessere Performance.
     */
    explicit GamecontrollerRobotBridge(ros::NodeHandle& node_handle, 
                                     const BridgeConfig& config);
    
    /**
     * @brief Destruktor
     * 
     * Sorgt für saubere Aufräumarbeiten und stoppt alle Roboter.
     * Der Destruktor ist virtual für korrekte Vererbung.
     */
    virtual ~GamecontrollerRobotBridge();
    
    /**
     * @brief Startet die Bridge-Funktionalität
     * @return true bei Erfolg, false bei Fehlern
     * 
     * Diese Methode startet alle Timer und aktiviert die Steuerung.
     */
    bool start();
    
    /**
     * @brief Stoppt die Bridge-Funktionalität sicher
     * 
     * Stoppt alle Roboter und deaktiviert die Steuerung.
     */
    void stop();
    
    /**
     * @brief Überprüft den aktuellen Status der Bridge
     * @return true wenn die Bridge aktiv und bereit ist
     */
    bool isActive() const { return is_active_.load(); }
    
    /**
     * @brief Gibt die ID des aktuell gesteuerten Roboters zurück
     * @return Aktuelle Roboter-ID
     */
    int getCurrentRobotId() const;
    
    /**
     * @brief Aktualisiert die Konfiguration zur Laufzeit
     * @param new_config Neue Konfiguration
     * @return true bei Erfolg
     * 
     * Diese Methode ermöglicht Live-Updates der Parameter.
     */
    bool updateConfig(const BridgeConfig& new_config);

private:
    // ===== Private Member Variablen =====
    
    /// ROS NodeHandle für diesen Node
    ros::NodeHandle& nh_;
    
    /// Aktuelle Konfiguration (thread-safe kopiert)
    BridgeConfig config_;
    
    /// Aktuell ausgewählter Roboter (atomic für thread safety)
    std::atomic<int> current_robot_id_{0};
    
    /// Status der Gamepad-Verbindung
    std::atomic<bool> gamepad_connected_{false};
    
    /// Bridge-Aktivitätsstatus
    std::atomic<bool> is_active_{false};
    
    /// Zeitpunkt der letzten Gamecontroller-Nachricht
    ros::Time last_joy_msg_time_;
    
    /// Mutex für thread-safe Zugriff auf kritische Bereiche
    mutable std::mutex state_mutex_;
    
    /// ROS Subscriber für Gamecontroller-Input
    ros::Subscriber joy_subscriber_;
    
    /// Map von Roboter-IDs zu ihren Publishern
    /// Wir verwenden shared_ptr für sichere Speicherverwaltung
    std::map<int, std::shared_ptr<ros::Publisher>> robot_publishers_;
    
    /// Timer für Verbindungsüberwachung
    ros::Timer connection_check_timer_;
    
    /// Button-Zustandsspeicher für Edge-Detection
    /// Das verhindert mehrfache Triggerung bei gehaltenen Buttons
    mutable std::mutex button_state_mutex_;
    std::vector<bool> previous_button_states_;
    
    // ===== Private Methoden =====
    
    /**
     * @brief Initialisiert alle Roboter-Publisher
     * 
     * Erstellt für jeden konfigurierten Roboter einen eigenen Publisher.
     * Verwendet RAII-Prinzipien für sichere Ressourcenverwaltung.
     */
    void setupRobotPublishers();
    
    /**
     * @brief Callback für Gamecontroller-Nachrichten
     * @param msg Eingehende Gamecontroller-Nachricht
     * 
     * Dies ist das Herzstück der Steuerungslogik. Die Methode wird
     * von ROS bei jeder neuen Gamecontroller-Nachricht aufgerufen.
     */
    void gamecontrollerCallback(const sensor_msgs::Joy::ConstPtr& msg);
    
    /**
     * @brief Verarbeitet Roboter-Auswahl basierend auf Button-Eingaben
     * @param msg Gamecontroller-Nachricht
     * 
     * Implementiert Edge-Detection für saubere Button-Behandlung.
     */
    void handleRobotSelection(const sensor_msgs::Joy::ConstPtr& msg);
    
    /**
     * @brief Verarbeitet Bewegungssteuerung basierend auf Joystick-Eingaben
     * @param msg Gamecontroller-Nachricht
     * 
     * Transformiert Joystick-Werte in Roboter-Bewegungskommandos.
     */
    void handleMovementControl(const sensor_msgs::Joy::ConstPtr& msg);
    
    /**
     * @brief Wendet Deadzone und Skalierung auf einen Joystick-Wert an
     * @param value Roher Joystick-Wert (-1.0 bis 1.0)
     * @param deadzone Deadzone-Schwellwert
     * @param scale Skalierungsfaktor
     * @return Verarbeiteter Wert
     * 
     * Diese Methode ist als inline definiert für maximale Performance
     * bei häufigen Aufrufen.
     */
    inline double applyDeadzoneAndScaling(double value, double deadzone, double scale) const;
    
    /**
     * @brief Sendet Geschwindigkeitskommando an spezifischen Roboter
     * @param robot_id Ziel-Roboter ID
     * @param linear_x Geschwindigkeit in X-Richtung
     * @param linear_y Geschwindigkeit in Y-Richtung  
     * @param angular_z Winkelgeschwindigkeit um Z-Achse
     * @return true bei Erfolg
     */
    bool sendVelocityToRobot(int robot_id, double linear_x, double linear_y, double angular_z);
    
    /**
     * @brief Wechselt den aktuell gesteuerten Roboter
     * @param direction +1 für nächsten, -1 für vorherigen Roboter
     * @return true bei Erfolg
     */
    bool switchRobot(int direction);
    
    /**
     * @brief Stoppt alle Roboter durch Setzen der Geschwindigkeiten auf 0
     * 
     * Diese Methode wird in Notfällen und beim Shutdown aufgerufen.
     */
    void stopAllRobots();
    
    /**
     * @brief Timer-Callback für Gamepad-Verbindungsüberwachung
     * @param event Timer-Event
     * 
     * Überwacht die Verbindung und stoppt Roboter bei Verbindungsverlust.
     */
    void checkGamepadConnection(const ros::TimerEvent& event);
    
    /**
     * @brief Validiert eine Roboter-ID
     * @param robot_id Zu prüfende ID
     * @return true wenn ID gültig ist
     */
    inline bool isValidRobotId(int robot_id) const {
        return robot_id >= 0 && robot_id < config_.num_robots;
    }
    
    /**
     * @brief Hilfsmethode für sichere Array-Zugriffe
     * @param vec Vector zum Prüfen
     * @param index Index zum Prüfen
     * @return true wenn Index gültig ist
     */
    template<typename T>
    inline bool isValidIndex(const std::vector<T>& vec, size_t index) const {
        return index < vec.size();
    }
};

/**
 * @class ConfigManager
 * @brief Hilfklasse für Konfigurationsmanagement
 * 
 * Diese Klasse kapselt das Laden und Validieren von Konfigurationsdateien.
 * Getrennte Klasse für bessere Testbarkeit und Wiederverwendbarkeit.
 */
class ConfigManager {
public:
    /**
     * @brief Lädt Konfiguration aus YAML-Datei
     * @param config_file Pfad zur Konfigurationsdatei
     * @return Geladene Konfiguration oder Default-Werte bei Fehlern
     */
    static BridgeConfig loadConfig(const std::string& config_file = "");
    
    /**
     * @brief Validiert eine Konfiguration
     * @param config Zu validierende Konfiguration
     * @return true wenn Konfiguration gültig ist
     */
    static bool validateConfig(const BridgeConfig& config);
    
    /**
     * @brief Gibt Default-Konfiguration zurück
     * @return Standard-Konfigurationsobjekt
     */
    static BridgeConfig getDefaultConfig();

private:
    /**
     * @brief Lädt Konfiguration aus YAML-Node
     * @param yaml_node YAML-Konfigurationsdaten
     * @param config Ziel-Konfigurationsobjekt
     */
    static void loadFromYaml(const YAML::Node& yaml_node, BridgeConfig& config);
};

} // namespace gamecontroller_bridge

#endif // GAMECONTROLLER_ROBOT_BRIDGE_H