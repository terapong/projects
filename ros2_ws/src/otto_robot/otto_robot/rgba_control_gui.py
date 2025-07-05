#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import ColorRGBA

# class RGBAControlGUI(Node):
#     def __init__(self):
#         super().__init__('rgba_control_gui')
#         self.publisher = self.create_publisher(ColorRGBA, 'led_color', 10)
        
#     def set_color(self, r, g, b, a=1.0):
#         msg = ColorRGBA()
#         msg.r = float(r)
#         msg.g = float(g)
#         msg.b = float(b)
#         msg.a = float(a)
#         self.publisher.publish(msg)
#         self.get_logger().info(f'Setting color: R={r}, G={g}, B={b}')

# def main(args=None):
#     rclpy.init(args=args)
#     controller = RGBAControlGUI()
    
#     try:
#         while True:
#             # Get user input
#             print("\nEnter RGB values (0-1) separated by spaces, or 'q' to quit:")
#             user_input = input("> ")
            
#             if user_input.lower() == 'q':
#                 break
                
#             try:
#                 r, g, b = map(float, user_input.split())
#                 # Validate input
#                 if all(0 <= x <= 1 for x in (r, g, b)):
#                     controller.set_color(r, g, b)
#                 else:
#                     print("Values must be between 0 and 1")
#             except ValueError:
#                 print("Invalid input. Please enter three numbers separated by spaces.")
                
#     except KeyboardInterrupt:
#         pass
        
#     controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# # 3. Run the Python script: `python3 led_controller.py`
# # 4. Enter RGB values when prompted (e.g., "1 0 0" for red, "0 1 0" for green, etc.)


import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import ColorRGBA
from tkinter.colorchooser import askcolor

class RGBAControlGUI(Node):
    def __init__(self, root):
        super().__init__('rgba_control_gui')
        
        self.publisher = self.create_publisher(ColorRGBA, 'led_color', 10)
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("RGBA Control")
        self.root.geometry("200x80")
    
        self.button = tk.Button(self.root, text="Setting color", command=self.choose_color).grid(row=0, column=0)
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=1, column=0)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
    
    # def select_color():
    # color = colorchooser.askcolor(title="Choose a Color")
    # if color[1]:  # If a color is selected (not canceled)
    #     rgb, hex_code = color
    #     print(f"RGB: {rgb}\nHex: {hex_code}")
    
    def choose_color(self):     
        color = askcolor(title="Choose a color")
        if color[1]:  # color[1] is the hex string, e.g., '#ff0000'
            # rgb = color
            # print("Selected color:", color[0][0])
            # print("Selected color:", color[0][1])
            # print("Selected color:", color[0][2])
            # print(f"RGB: {rgb}")
            msg = ColorRGBA()
            msg.r = float(color[0][0])
            msg.g = float(color[0][1])
            msg.b = float(color[0][2])
            msg.a = float(1.0)
            self.publisher.publish(msg)
            self.get_logger().info(f'Setting color: R={msg.r}, G={msg.g}, B={msg.b}')
    
    def listener_callback(self, msg):
        color = msg.data
        # r, g, b = map(float, user_input.split())
        
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
# def main(args=None):
#     rclpy.init(args=args)
#     root = tk.Tk()
#     controller = RGBAControlGUI(root)
    
#     try:
#         while True:
#             # Get user input
#             print("\nEnter RGB values (0-1) separated by spaces, or 'q' to quit:")
#             user_input = input("> ")
            
#             if user_input.lower() == 'q':
#                 break
                
#             try:
#                 r, g, b = map(float, user_input.split())
#                 # Validate input
#                 if all(0 <= x <= 1 for x in (r, g, b)):
#                     controller.set_color(r, g, b)
#                 else:
#                     print("Values must be between 0 and 1")
#             except ValueError:
#                 print("Invalid input. Please enter three numbers separated by spaces.")
                
#     except KeyboardInterrupt:
#         pass
        
#     controller.destroy_node()
#     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = RGBAControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()