#!/usr/bin/env python3
"""
Gamecontroller zu Roboter Bridge Node

Dieser Knoten verbindet Gamecontroller-Eingaben mit Roboter-Motorsteuerung.
Er ermöglicht die Steuerung eines oder mehrerer Roboter über einen Gamecontroller.

Autor: Senior-Entwickler
Version: 1.0
"""

import rospy
import yaml
import argparse
from threading import Lock
from typing import Dict, List, Optional

# Beispiel-Nachrichtentypen (diese müssen durch echte Typen von #8 und #12 ersetzt werden)
from sensor_msgs.msg import Joy  # Gamecontroller Input (Beispiel)
from geometry_msgs.msg import Twist  # Roboter Bewegungskommandos (Beispiel)
from std_msgs.msg import Int32  # Für Roboter-Auswahl


class GamecontrollerRobotBridge:
    """
    Hauptklasse für die Gamecontroller-Roboter-Brücke.
    
    Diese Klasse implementiert die gesamte Logik für:
    - Gamecontroller-Input-Verarbeitung
    - Multi-Roboter-Verwaltung
    - Sichere Geschwindigkeitssteuerung
    """
    
    def __init__(self, config: Dict):
        """
        Initialisiert den Bridge-Knoten mit der gegebenen Konfiguration.
        
        Args:
            config: Konfigurationswörterbuch mit allen Parametern
        """
        self.config = config
        self.current_robot_id = 0  # Aktuell ausgewählter Roboter
        self.gamepad_connected = False
        self.last_joy_msg_time = rospy.Time.now()
        
        # Thread-sichere Sperre für Zustandsänderungen
        self.state_lock = Lock()
        
        # Initialisiere ROS Publishers für jeden Roboter
        self.robot_publishers: Dict[int, rospy.Publisher] = {}
        self._setup_robot_publishers()
        
        # Gamecontroller Subscriber
        self.joy_subscriber = rospy.Subscriber(
            self.config['gamecontroller_topic'],
            Joy,  # Dieser Typ muss durch den echten Typ von #8 ersetzt werden
            self._gamecontroller_callback,
            queue_size=1
        )
        
        # Timer für Verbindungsüberwachung
        self.connection_check_timer = rospy.Timer(
            rospy.Duration(self.config['connection_timeout']),
            self._check_gamepad_connection
        )
        
        # Initialisiere alle Roboter-Geschwindigkeiten auf 0
        self._stop_all_robots()
        
        rospy.loginfo(f"Gamecontroller Bridge Node gestartet. "
                     f"Steuert {len(self.robot_publishers)} Roboter.")
    
    def _setup_robot_publishers(self):
        """
        Erstellt Publisher für alle konfigurierten Roboter.
        
        Diese Methode initialisiert die ROS-Publisher basierend auf der
        Konfiguration. Jeder Roboter bekommt seinen eigenen Publisher.
        """
        for robot_id in range(self.config['num_robots']):
            topic_name = f"{self.config['robot_control_topic_prefix']}_robot_{robot_id}"
            
            publisher = rospy.Publisher(
                topic_name,
                Twist,  # Dieser Typ muss durch den echten Typ von #12 ersetzt werden
                queue_size=1
            )
            
            self.robot_publishers[robot_id] = publisher
            rospy.loginfo(f"Publisher für Roboter {robot_id} erstellt: {topic_name}")
    
    def _gamecontroller_callback(self, msg):
        """
        Callback-Funktion für Gamecontroller-Nachrichten.
        
        Diese Funktion wird bei jeder neuen Gamecontroller-Nachricht aufgerufen
        und implementiert die gesamte Steuerungslogik.
        
        Args:
            msg: Gamecontroller-Nachricht (Joy oder entsprechender Typ von #8)
        """
        with self.state_lock:
            # Aktualisiere Verbindungsstatus
            self.gamepad_connected = True
            self.last_joy_msg_time = rospy.Time.now()
            
            # Verarbeite Roboter-Auswahl (Buttons)
            self._handle_robot_selection(msg)
            
            # Verarbeite Bewegungssteuerung (Joysticks)
            self._handle_movement_control(msg)
    
    def _handle_robot_selection(self, msg):
        """
        Behandelt die Roboter-Auswahl über Gamepad-Buttons.
        
        Diese Methode ermöglicht es, zwischen verschiedenen Robotern zu wechseln.
        
        Args:
            msg: Gamecontroller-Nachricht
        """
        # Beispiel: Verwendung der Schultertasten für Roboter-Auswahl
        # Button-Mapping kann in der Konfiguration angepasst werden
        
        if hasattr(msg, 'buttons') and len(msg.buttons) > 4:
            # L1 (Button 4) - Vorheriger Roboter
            if msg.buttons[4] and not hasattr(self, '_l1_pressed'):
                self._switch_robot(-1)
                self._l1_pressed = True
            elif not msg.buttons[4]:
                self._l1_pressed = False
            
            # R1 (Button 5) - Nächster Roboter  
            if msg.buttons[5] and not hasattr(self, '_r1_pressed'):
                self._switch_robot(1)
                self._r1_pressed = True
            elif not msg.buttons[5]:
                self._r1_pressed = False
    
    def _switch_robot(self, direction: int):
        """
        Wechselt den aktuell gesteuerten Roboter.
        
        Args:
            direction: +1 für nächsten, -1 für vorherigen Roboter
        """
        old_robot = self.current_robot_id
        
        # Berechne neue Roboter-ID mit Wraparound
        self.current_robot_id = (self.current_robot_id + direction) % self.config['num_robots']
        
        # Stoppe den vorherigen Roboter
        self._send_velocity_to_robot(old_robot, 0.0, 0.0, 0.0)
        
        rospy.loginfo(f"Roboter-Auswahl gewechselt von {old_robot} zu {self.current_robot_id}")
    
    def _handle_movement_control(self, msg):
        """
        Verarbeitet die Bewegungssteuerung basierend auf Joystick-Eingaben.
        
        Diese Methode implementiert die Hauptsteuerungslogik:
        - Linker Joystick: X/Y-Bewegung
        - Rechter Joystick: Rotation
        
        Args:
            msg: Gamecontroller-Nachricht
        """
        if not hasattr(msg, 'axes') or len(msg.axes) < 4:
            rospy.logwarn("Gamecontroller-Nachricht hat nicht genügend Achsen")
            return
        
        # Extrahiere Joystick-Werte (normalerweise im Bereich -1.0 bis 1.0)
        left_stick_x = msg.axes[0]   # Links/Rechts Bewegung
        left_stick_y = msg.axes[1]   # Vor/Zurück Bewegung  
        right_stick_x = msg.axes[2]  # Rotation (Yaw)
        
        # Anwenden von Deadzone und Skalierung
        linear_x = self._apply_deadzone_and_scaling(
            left_stick_x, 
            self.config['deadzone'],
            self.config['linear_scale_factor']
        )
        
        linear_y = self._apply_deadzone_and_scaling(
            left_stick_y,
            self.config['deadzone'], 
            self.config['linear_scale_factor']
        )
        
        angular_z = self._apply_deadzone_and_scaling(
            right_stick_x,
            self.config['deadzone'],
            self.config['angular_scale_factor']
        )
        
        # Invertiere Rotation für intuitive Steuerung (rechts = im Uhrzeigersinn)
        angular_z = -angular_z
        
        # Sende Geschwindigkeitskommando an aktuellen Roboter
        self._send_velocity_to_robot(self.current_robot_id, linear_x, linear_y, angular_z)
        
        # Stoppe alle anderen Roboter
        for robot_id in self.robot_publishers.keys():
            if robot_id != self.current_robot_id:
                self._send_velocity_to_robot(robot_id, 0.0, 0.0, 0.0)
    
    def _apply_deadzone_and_scaling(self, value: float, deadzone: float, scale: float) -> float:
        """
        Wendet Deadzone und Skalierung auf einen Joystick-Wert an.
        
        Die Deadzone verhindert ungewollte Bewegungen bei leichten Joystick-Berührungen.
        Die Skalierung ermöglicht es, die Geschwindigkeit anzupassen.
        
        Args:
            value: Roher Joystick-Wert (-1.0 bis 1.0)
            deadzone: Deadzone-Schwellwert (0.0 bis 1.0)
            scale: Skalierungsfaktor für die Ausgabe
            
        Returns:
            Verarbeiteter Wert
        """
        # Anwenden der Deadzone
        if abs(value) < deadzone:
            return 0.0
        
        # Lineare Skalierung außerhalb der Deadzone
        sign = 1.0 if value >= 0 else -1.0
        scaled_value = (abs(value) - deadzone) / (1.0 - deadzone)
        
        return sign * scaled_value * scale
    
    def _send_velocity_to_robot(self, robot_id: int, linear_x: float, linear_y: float, angular_z: float):
        """
        Sendet ein Geschwindigkeitskommando an einen spezifischen Roboter.
        
        Args:
            robot_id: ID des Zielroboters
            linear_x: Geschwindigkeit in X-Richtung (links/rechts)
            linear_y: Geschwindigkeit in Y-Richtung (vor/zurück)
            angular_z: Winkelgeschwindigkeit um Z-Achse (Rotation)
        """
        if robot_id not in self.robot_publishers:
            rospy.logwarn(f"Unbekannte Roboter-ID: {robot_id}")
            return
        
        # Erstelle Geschwindigkeitsnachricht
        # WICHTIG: Dieser Nachrichtentyp muss durch den echten Typ von #12 ersetzt werden
        velocity_msg = Twist()
        velocity_msg.linear.x = linear_x
        velocity_msg.linear.y = linear_y
        velocity_msg.linear.z = 0.0
        velocity_msg.angular.x = 0.0
        velocity_msg.angular.y = 0.0
        velocity_msg.angular.z = angular_z
        
        # Veröffentliche die Nachricht
        self.robot_publishers[robot_id].publish(velocity_msg)
        
        # Debug-Ausgabe (nur bei Bewegung)
        if abs(linear_x) > 0.01 or abs(linear_y) > 0.01 or abs(angular_z) > 0.01:
            rospy.logdebug(f"Roboter {robot_id}: vx={linear_x:.2f}, vy={linear_y:.2f}, wz={angular_z:.2f}")
    
    def _stop_all_robots(self):
        """
        Stoppt alle Roboter durch Setzen ihrer Geschwindigkeiten auf 0.
        
        Diese Methode wird beim Start und bei Verbindungsverlust aufgerufen.
        """
        for robot_id in self.robot_publishers.keys():
            self._send_velocity_to_robot(robot_id, 0.0, 0.0, 0.0)
        
        rospy.loginfo("Alle Roboter gestoppt")
    
    def _check_gamepad_connection(self, event):
        """
        Timer-Callback zur Überwachung der Gamepad-Verbindung.
        
        Wenn für eine bestimmte Zeit keine Nachrichten empfangen wurden,
        wird angenommen, dass das Gamepad getrennt wurde.
        
        Args:
            event: Timer-Event (nicht verwendet)
        """
        with self.state_lock:
            time_since_last_msg = rospy.Time.now() - self.last_joy_msg_time
            
            if time_since_last_msg.to_sec() > self.config['connection_timeout']:
                if self.gamepad_connected:
                    rospy.logwarn("Gamepad-Verbindung verloren! Stoppe alle Roboter.")
                    self._stop_all_robots()
                    self.gamepad_connected = False


def load_config(config_file: Optional[str] = None) -> Dict:
    """
    Lädt die Konfiguration aus einer YAML-Datei oder verwendet Standardwerte.
    
    Args:
        config_file: Pfad zur Konfigurationsdatei (optional)
        
    Returns:
        Konfigurationswörterbuch
    """
    # Standardkonfiguration
    default_config = {
        'gamecontroller_topic': '/joy',  # Topic von Knoten #8
        'robot_control_topic_prefix': '/robot_cmd_vel',  # Topic-Präfix für Knoten #12
        'num_robots': 3,  # Anzahl der zu steuernden Roboter
        'deadzone': 0.1,  # Joystick-Deadzone (0.0-1.0)
        'linear_scale_factor': 1.0,  # Skalierung für lineare Geschwindigkeit
        'angular_scale_factor': 2.0,  # Skalierung für Winkelgeschwindigkeit
        'connection_timeout': 1.0,  # Timeout für Verbindungsüberwachung (Sekunden)
    }
    
    if config_file:
        try:
            with open(config_file, 'r') as f:
                file_config = yaml.safe_load(f)
                # Überschreibe Standardwerte mit Datei-Konfiguration
                default_config.update(file_config)
                rospy.loginfo(f"Konfiguration geladen aus: {config_file}")
        except Exception as e:
            rospy.logwarn(f"Fehler beim Laden der Konfigurationsdatei {config_file}: {e}")
            rospy.loginfo("Verwende Standardkonfiguration")
    
    return default_config


def parse_arguments():
    """
    Parst Kommandozeilenargumente.
    
    Returns:
        Argparse-Namespace mit den geparsten Argumenten
    """
    parser = argparse.ArgumentParser(description='Gamecontroller zu Roboter Bridge Node')
    
    parser.add_argument(
        '--config', '-c',
        type=str,
        help='Pfad zur Konfigurationsdatei (YAML)'
    )
    
    parser.add_argument(
        '--num-robots', '-n',
        type=int,
        help='Anzahl der zu steuernden Roboter'
    )
    
    parser.add_argument(
        '--linear-scale', '-l',
        type=float,
        help='Skalierungsfaktor für lineare Geschwindigkeit'
    )
    
    parser.add_argument(
        '--angular-scale', '-a',
        type=float,
        help='Skalierungsfaktor für Winkelgeschwindigkeit'
    )
    
    return parser.parse_args()


def main():
    """
    Hauptfunktion - Initialisiert und startet den Bridge-Knoten.
    """
    # Initialisiere ROS-Knoten
    rospy.init_node('gamecontroller_robot_bridge', anonymous=False)
    
    # Parse Kommandozeilenargumente
    args = parse_arguments()
    
    # Lade Konfiguration
    config = load_config(args.config)
    
    # Überschreibe Konfiguration mit Kommandozeilenargumenten
    if args.num_robots is not None:
        config['num_robots'] = args.num_robots
    if args.linear_scale is not None:
        config['linear_scale_factor'] = args.linear_scale
    if args.angular_scale is not None:
        config['angular_scale_factor'] = args.angular_scale
    
    # Erstelle und starte Bridge-Knoten
    try:
        bridge = GamecontrollerRobotBridge(config)
        rospy.loginfo("Gamecontroller Robot Bridge gestartet. Warte auf Eingaben...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Bridge-Knoten beendet")
    except Exception as e:
        rospy.logerr(f"Fehler im Bridge-Knoten: {e}")
        raise


if __name__ == '__main__':
    main()