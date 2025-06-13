/**
 * @file gamecontroller_bridge_main.cpp
 * @brief Hauptprogramm für den Gamecontroller Robot Bridge Node
 * 
 * Diese Datei enthält die main()-Funktion und Kommandozeilenargument-Parsing.
 * In C++ trennen wir oft die Klassen-Implementation von der main()-Funktion
 * für bessere Testbarkeit und Modularität.
 */

#include "gamecontroller_robot_bridge.h"
#include <ros/ros.h>
#include <signal.h>
#include <memory>

// Command-line argument parsing
#include <boost/program_options.hpp>
namespace po = boost::program_options;

// Global pointer für Signal Handler
std::shared_ptr<gamecontroller_bridge::GamecontrollerRobotBridge> g_bridge_ptr;

/**
 * @brief Signal Handler für sauberes Shutdown
 * @param sig Signal Nummer
 * 
 * Dieser Handler sorgt dafür, dass alle Roboter sicher gestoppt werden,
 * wenn das Programm durch Ctrl+C oder andere Signale beendet wird.
 */
void signalHandler(int sig) {
    ROS_INFO("Interrupt Signal (%d) empfangen, beende Bridge...", sig);
    
    if (g_bridge_ptr) {
        g_bridge_ptr->stop();
    }
    
    ros::shutdown();
}

/**
 * @brief Parst Kommandozeilenargumente
 * @param argc Anzahl Argumente
 * @param argv Argument Array
 * @return Pfad zur Konfigurationsdatei oder leer für Defaults
 */
std::string parseCommandLineArguments(int argc, char** argv) {
    std::string config_file;
    int num_robots = -1;
    double linear_scale = -1.0;
    double angular_scale = -1.0;
    
    try {
        po::options_description desc("Gamecontroller Robot Bridge Optionen");
        desc.add_options()
            ("help,h", "Zeige diese Hilfe")
            ("config,c", po::value<std::string>(&config_file), 
             "Pfad zur Konfigurationsdatei (YAML)")
            ("num-robots,n", po::value<int>(&num_robots), 
             "Anzahl der zu steuernden Roboter")
            ("linear-scale,l", po::value<double>(&linear_scale), 
             "Skalierungsfaktor für lineare Geschwindigkeit")
            ("angular-scale,a", po::value<double>(&angular_scale), 
             "Skalierungsfaktor für Winkelgeschwindigkeit")
            ("debug,d", "Aktiviere Debug-Ausgaben");
        
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
        
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            std::cout << "\nBeispiele:" << std::endl;
            std::cout << "  " << argv[0] << " --config ~/robot_config.yaml" << std::endl;
            std::cout << "  " << argv[0] << " --num-robots 5 --linear-scale 0.8" << std::endl;
            exit(0);
        }
        
        // Setze ROS-Parameter für die übergebenen Werte
        // Das ermöglicht es der Config-Klasse, diese zu lesen
        if (num_robots > 0) {
            ros::param::set("~num_robots", num_robots);
        }
        if (linear_scale > 0.0) {
            ros::param::set("~linear_scale_factor", linear_scale);
        }
        if (angular_scale > 0.0) {
            ros::param::set("~angular_scale_factor", angular_scale);
        }
        if (vm.count("debug")) {
            ros::param::set("~debug_output", true);
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("Fehler beim Parsen der Kommandozeilenargumente: %s", e.what());
        exit(1);
    }
    
    return config_file;
}

/**
 * @brief Erweiterte Config-Loader die auch ROS-Parameter berücksichtigt
 * @param config_file Pfad zur Basis-Konfigurationsdatei
 * @return Finale Konfiguration
 */
gamecontroller_bridge::BridgeConfig loadConfigWithRosParams(const std::string& config_file) {
    // Lade Basis-Konfiguration
    auto config = gamecontroller_bridge::ConfigManager::loadConfig(config_file);
    
    // Überschreibe mit ROS-Parametern falls vorhanden
    int num_robots;
    if (ros::param::get("~num_robots", num_robots) && num_robots > 0) {
        config.num_robots = num_robots;
        ROS_INFO("Anzahl Roboter von ROS-Parameter überschrieben: %d", num_robots);
    }
    
    double linear_scale;
    if (ros::param::get("~linear_scale_factor", linear_scale) && linear_scale > 0.0) {
        config.linear_scale_factor = linear_scale;
        ROS_INFO("Linearer Skalierungsfaktor von ROS-Parameter überschrieben: %.2f", linear_scale);
    }
    
    double angular_scale;
    if (ros::param::get("~angular_scale_factor", angular_scale) && angular_scale > 0.0) {
        config.angular_scale_factor = angular_scale;
        ROS_INFO("Angularer Skalierungsfaktor von ROS-Parameter überschrieben: %.2f", angular_scale);
    }
    
    bool debug_output;
    if (ros::param::get("~debug_output", debug_output)) {
        config.debug_velocity_output = debug_output;
        config.debug_robot_selection = debug_output;
    }
    
    // Weitere ROS-Parameter können hier gelesen werden
    std::string gamecontroller_topic;
    if (ros::param::get("~gamecontroller_topic", gamecontroller_topic)) {
        config.gamecontroller_topic = gamecontroller_topic;
    }
    
    std::string robot_control_prefix;
    if (ros::param::get("~robot_control_topic_prefix", robot_control_prefix)) {
        config.robot_control_topic_prefix = robot_control_prefix;
    }
    
    return config;
}

/**
 * @brief Hauptfunktion
 * @param argc Anzahl Kommandozeilenargumente
 * @param argv Kommandozeilenargument Array
 * @return Exit-Code (0 = Erfolg)
 */
int main(int argc, char** argv) {
    // Initialisiere ROS-Node
    // Der Node-Name wird automatisch eindeutig gemacht wenn mehrere Instanzen laufen
    ros::init(argc, argv, "gamecontroller_robot_bridge");
    
    // Erstelle NodeHandle
    ros::NodeHandle nh;           // Globaler namespace
    ros::NodeHandle nh_private("~");  // Private namespace für Parameter
    
    ROS_INFO("Starte Gamecontroller Robot Bridge Node...");
    
    try {
        // Parse Kommandozeilenargumente
        std::string config_file = parseCommandLineArguments(argc, argv);
        
        // Lade Konfiguration
        auto config = loadConfigWithRosParams(config_file);
        
        // Validiere finale Konfiguration
        if (!gamecontroller_bridge::ConfigManager::validateConfig(config)) {
            ROS_FATAL("Ungültige Konfiguration, kann nicht fortfahren");
            return 1;
        }
        
        // Erstelle Bridge-Instanz
        g_bridge_ptr = std::make_shared<gamecontroller_bridge::GamecontrollerRobotBridge>(
            nh, config
        );
        
        // Registriere Signal Handler für sauberes Shutdown
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        // Starte Bridge
        if (!g_bridge_ptr->start()) {
            ROS_FATAL("Konnte Bridge nicht starten");
            return 1;
        }
        
        ROS_INFO("Gamecontroller Robot Bridge erfolgreich gestartet");
        ROS_INFO("Steuert %d Roboter, aktuell ausgewählt: Roboter %d", 
                 config.num_robots, g_bridge_ptr->getCurrentRobotId());
        ROS_INFO("Verwende Gamecontroller Topic: %s", config.gamecontroller_topic.c_str());
        ROS_INFO("Verwende Roboter Control Prefix: %s", config.robot_control_topic_prefix.c_str());
        ROS_INFO("Bereit für Gamecontroller-Eingaben...");
        
        // Haupt-Event-Loop
        // ros::spin() blockiert bis ros::shutdown() aufgerufen wird
        ros::spin();
        
        ROS_INFO("Gamecontroller Robot Bridge beendet");
        
    } catch (const std::exception& e) {
        ROS_FATAL("Unbehandelter Fehler in main(): %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("Unbekannter Fehler in main()");
        return 1;
    }
    
    return 0;
}

/*
===== CMakeLists.txt für das Projekt =====

cmake_minimum_required(VERSION 3.0.2)
project(gamecontroller_robot_bridge)

## Kompiler-Einstellungen für C++14 (oder C++17 für modernere Features)
add_compile_options(-std=c++14)

## Finde ROS-Pakete
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
)

## Finde externe Abhängigkeiten
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Catkin-spezifische Konfiguration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs
  DEPENDS Boost YAML_CPP
)

## Include-Verzeichnisse
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Erstelle Bibliothek
add_library(${PROJECT_NAME}
  src/gamecontroller_robot_bridge.cpp
)

## Linke Bibliotheken
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

## Erstelle ausführbare Datei
add_executable(${PROJECT_NAME}_node src/gamecontroller_bridge_main.cpp)

## Linke ausführbare Datei
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Abhängigkeiten für Message-Generation (falls eigene Messages verwendet werden)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

===== package.xml =====

<?xml version="1.0"?>
<package format="2">
  <name>gamecontroller_robot_bridge</name>
  <version>1.0.0</version>
  <description>Bridge zwischen Gamecontroller-Input und Roboter-Motorsteuerung</description>

  <maintainer email="developer@example.com">Senior-Entwickler</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>libboost-program-options-dev</build_depend>
  <build_depend>libyaml-cpp-dev</build_depend>

  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>libboost-program-options</exec_depend>
  <exec_depend>libyaml-cpp0.6</exec_depend>

  <export>
  </export>
</package>

===== Launch-Datei (launch/gamecontroller_bridge.launch) =====

<launch>
  <!-- Gamecontroller Robot Bridge Node -->
  <node name="gamecontroller_robot_bridge" pkg="gamecontroller_robot_bridge" type="gamecontroller_robot_bridge_node" output="screen">
    
    <!-- Basis-Konfiguration -->
    <param name="gamecontroller_topic" value="/joy" />
    <param name="robot_control_topic_prefix" value="/robot_cmd_vel" />
    <param name="num_robots" value="3" />
    
    <!-- Steuerungsparameter -->
    <param name="deadzone" value="0.15" />
    <param name="linear_scale_factor" value="1.5" />
    <param name="angular_scale_factor" value="2.0" />
    <param name="connection_timeout" value="2.0" />
    
    <!-- Debug-Einstellungen -->
    <param name="debug_output" value="false" />
    
    <!-- Optionale Konfigurationsdatei -->
    <!-- <param name="config_file" value="$(find gamecontroller_robot_bridge)/config/bridge_config.yaml" /> -->
    
  </node>

  <!-- Optional: Starte auch joy_node für Gamecontroller-Input -->
  <!-- <node name="joy_node" pkg="joy" type="joy_node" output="screen">
    <param name="dev" value="/dev/input/js0" />
    <param name="deadzone" value="0.05" />
    <param name="autorepeat_rate" value="20" />
  </node> -->

</launch>
*/