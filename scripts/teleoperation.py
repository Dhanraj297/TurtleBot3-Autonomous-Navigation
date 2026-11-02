"""
TurtleBot3 Keyboard Teleoperation Node
Robotics Laboratory ESE

Controls the simulated (Pygame / Gazebo) or real TurtleBot3 using standard terminal keypresses,
publishing geometry_msgs/Twist messages to the /cmd_vel topic.
Compatible with both Windows (msvcrt) and Linux / WSL (termios).

Keybindings:
---------------------------
   Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (0.02 m/s step)
a/d : increase/decrease angular velocity (0.1 rad/s step)
space / s : force immediate emergency stop
q : quit teleoperation
"""

import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import rclpy
    from geometry_msgs.msg import Twist
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def print_teleop_banner(v: float, omega: float, max_v: float = 0.22, max_w: float = 2.84):
    os.system('cls' if os.name == 'nt' else 'clear')
    print("=" * 58)
    print("      TURTLEBOT3 KEYBOARD TELEOPERATION (ROS 2)      ")
    print("=" * 58)
    print("  Moving around:")
    print("        w")
    print("   a    s    d")
    print("        x\n")
    print("  w/x : increase/decrease linear velocity (+/- 0.02 m/s)")
    print("  a/d : increase/decrease angular velocity (+/- 0.1 rad/s)")
    print("  space / s : force stop")
    print("  q : quit\n")
    print(f"  CURRENT SPEED:")
    print(f"    Linear Velocity (v)  : {v:+.2f} m/s  (max: +/-{max_v:.2f})")
    print(f"    Angular Velocity (w) : {omega:+.2f} rad/s (max: +/-{max_w:.2f})")
    print("=" * 58)


class NonBlockingKeyReader:
    """Cross-platform non-blocking key reader for Windows and POSIX (Linux / WSL)."""

    def __init__(self):
        self.is_windows = (sys.platform == "win32")
        if not self.is_windows:
            import termios
            import tty
            self.termios = termios
            self.tty = tty
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def read_char(self):
        if self.is_windows:
            import msvcrt
            if msvcrt.kbhit():
                try:
                    return msvcrt.getch().decode("utf-8").lower()
                except UnicodeDecodeError:
                    return None
            return None
        else:
            import select
            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist:
                return sys.stdin.read(1).lower()
            return None

    def restore(self):
        if not self.is_windows and hasattr(self, 'old_settings'):
            self.termios.tcsetattr(sys.stdin, self.termios.TCSADRAIN, self.old_settings)


def main():
    node = None
    pub = None

    if ROS2_AVAILABLE:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("turtlebot3_teleop_keyboard")
        pub = node.create_publisher(Twist, "/cmd_vel", 10)
        print("[ROS 2] Initialized teleop publisher on /cmd_vel")

    target_v = 0.0
    target_omega = 0.0
    max_v = 0.22
    max_omega = 2.84
    step_v = 0.02
    step_omega = 0.15

    print_teleop_banner(target_v, target_omega)
    key_reader = NonBlockingKeyReader()

    try:
        while True:
            char = key_reader.read_char()
            if char:
                if char == 'w':
                    target_v = min(max_v, target_v + step_v)
                elif char == 'x':
                    target_v = max(-max_v, target_v - step_v)
                elif char == 'a':
                    target_omega = min(max_omega, target_omega + step_omega)
                elif char == 'd':
                    target_omega = max(-max_omega, target_omega - step_omega)
                elif char == 's' or char == ' ':
                    target_v = 0.0
                    target_omega = 0.0
                elif char == 'q':
                    print("\nExiting teleoperation...")
                    break

                print_teleop_banner(target_v, target_omega)

                # Publish ROS 2 twist
                if pub and node:
                    twist = Twist()
                    twist.linear.x = float(target_v)
                    twist.angular.z = float(target_omega)
                    pub.publish(twist)

            time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        key_reader.restore()
        if pub and node:
            twist = Twist()
            pub.publish(twist)  # Stop robot on exit
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
