import tkinter as tk

from gui import TkinterApp
import rclpy
from ros2_interface import ROS2Interface


class Main:
    running_state = True
    ros_interface_node = None
    root = None

    def cleanup(self):
        if self.ros_interface_node:
            self.ros_interface_node.cleanup()
            self.ros_interface_node.destroy_node()
        if self.root:
            self.root.destroy()
        self.running_state = False

    def main(self, args=None):
        rclpy.init(args=args)
        root = tk.Tk()
        root.protocol("WM_DELETE_WINDOW", self.cleanup)
        ros_interface_node = ROS2Interface(node_name="trajectory_visualizer")
        try:
            app = TkinterApp(root, ros_interface_node)
            while self.running_state:
                root.update_idletasks()
                root.update()
                rclpy.spin_once(ros_interface_node, timeout_sec=0.1)
                app.replot()
        except KeyboardInterrupt:
            ros_interface_node.get_logger().info("KeyboardInterrupt received, shutting down.")
        finally:
            self.cleanup()


if __name__ == "__main__":
    # TODO: provide initial topics as arguments
    m = Main()
    m.main()
