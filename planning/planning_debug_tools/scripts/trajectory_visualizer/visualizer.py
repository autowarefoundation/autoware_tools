import tkinter as tk
from gui import TkinterApp
import rclpy
from ros2_interface import ROS2Interface

def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    ros_interface_node = ROS2Interface(node_name="trajectory_visualizer")
    try:
        app = TkinterApp(root, ros_interface_node)
        while True:
            root.update_idletasks()
            root.update()
            rclpy.spin_once(ros_interface_node, timeout_sec=0.1)
            app.replot()
    except KeyboardInterrupt:
        ros_interface_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        if ros_interface_node:
            ros_interface_node.cleanup()
            ros_interface_node.destroy_node()
        print("ROS 2 Interface shut down cleanly.")

if __name__ == "__main__":
    main()