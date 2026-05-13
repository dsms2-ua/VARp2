#!/usr/bin/env python3
"""
Teleoperación + grabación de waypoints para el circuito.

Controles:
  W / S        — aumentar / reducir velocidad lineal
  A / D        — girar izquierda / derecha
  Espacio      — parar (velocidad 0)
  M            — guardar waypoint en la posición actual
  Z            — deshacer último waypoint guardado
  P            — imprimir lista de waypoints actuales
  Q            — guardar archivo YAML y salir

El archivo se guarda en: waypoints/circuit_waypoints.yaml
"""

import sys
import os
import math
import termios
import tty
import select
import threading
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


BANNER = """
=== TELEOP + WAYPOINT RECORDER ===
  W/S  : velocidad lineal  (+/-)
  A/D  : giro izquierda / derecha
  SPACE: parar
  M    : guardar waypoint aquí
  Z    : deshacer último waypoint
  P    : mostrar waypoints
  Q    : guardar YAML y salir
==================================
"""

LINEAR_STEP  = 0.02   # m/s por pulsación
ANGULAR_STEP = 0.1    # rad/s por pulsación
MAX_LINEAR   = 0.26
MAX_ANGULAR  = 1.82


def get_key(timeout=0.1):
    """Lee una tecla sin bloquear (devuelve '' si no hay tecla)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            key = sys.stdin.read(1)
            # Secuencias de escape (flechas, etc.) — descartar
            if key == '\x1b':
                select.select([sys.stdin], [], [], 0.05)
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.read(2)
                key = ''
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        self._pos = (0.0, 0.0, 0.0)   # x, y, yaw
        self._lock = threading.Lock()
        self._waypoints = []
        self._lin = 0.0
        self._ang = 0.0

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            self._pos = (p.x, p.y, yaw)

    def current_pos(self):
        with self._lock:
            return self._pos

    def publish_vel(self):
        msg = Twist()
        msg.linear.x  = self._lin
        msg.angular.z = self._ang
        self.pub_vel.publish(msg)

    def stop(self):
        self._lin = 0.0
        self._ang = 0.0
        self.publish_vel()

    def save_waypoint(self, output_path):
        x, y, yaw = self.current_pos()
        wp = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        self._waypoints.append(wp)
        idx = len(self._waypoints) - 1
        self.save_yaml(output_path)
        print(f"  [WP {idx:02d}] x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°  → guardado")

    def undo_waypoint(self):
        if self._waypoints:
            removed = self._waypoints.pop()
            print(f"  Deshecho: {removed}")
        else:
            print("  No hay waypoints que deshacer.")

    def print_waypoints(self):
        if not self._waypoints:
            print("  (ningún waypoint guardado todavía)")
            return
        print(f"  --- {len(self._waypoints)} waypoints ---")
        for i, wp in enumerate(self._waypoints):
            print(f"  [{i:02d}] x={wp['x']:.3f}  y={wp['y']:.3f}  "
                  f"yaw={math.degrees(wp['yaw']):.1f}°")

    def save_yaml(self, path):
        data = {'waypoints': self._waypoints}
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
        print(f"\n  Guardados {len(self._waypoints)} waypoints en:\n  {path}")


def main():
    rclpy.init()
    node = WaypointRecorder()

    # Spin ROS en hilo separado
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Ruta de salida
    script_dir = os.path.dirname(os.path.abspath(__file__))
    waypoints_dir = os.path.join(script_dir, '..', 'waypoints')
    output_path = os.path.join(waypoints_dir, 'circuit_waypoints.yaml')

    print(BANNER)

    try:
        while rclpy.ok():
            key = get_key(timeout=0.05)

            if key == 'w':
                node._lin = min(node._lin + LINEAR_STEP, MAX_LINEAR)
            elif key == 's':
                node._lin = max(node._lin - LINEAR_STEP, -MAX_LINEAR)
            elif key == 'a':
                node._ang = min(node._ang + ANGULAR_STEP, MAX_ANGULAR)
            elif key == 'd':
                node._ang = max(node._ang - ANGULAR_STEP, -MAX_ANGULAR)
            elif key == ' ':
                node._lin = 0.0
                node._ang = 0.0
            elif key == 'm':
                node.save_waypoint(output_path)
            elif key == 'z':
                node.undo_waypoint()
            elif key == 'p':
                node.print_waypoints()
            elif key in ('q', 'Q', '\x03'):  # Q o Ctrl+C
                node.stop()
                node.save_yaml(output_path)
                break

            node.publish_vel()

            x, y, yaw = node.current_pos()
            sys.stdout.write(
                f"\r  pos=({x:6.3f}, {y:6.3f})  yaw={math.degrees(yaw):6.1f}°"
                f"  lin={node._lin:.2f}  ang={node._ang:.2f}  WPs={len(node._waypoints)}  "
            )
            sys.stdout.flush()

    except KeyboardInterrupt:
        node.stop()
        node.save_yaml(output_path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
