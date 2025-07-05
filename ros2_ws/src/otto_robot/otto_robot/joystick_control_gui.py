#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class JoystickControlGUI(Node):
    def __init__(self, root):
        super().__init__('joystick_control_gui')
        
        self.subscription = self.create_subscription(Joy, 'joystick_input', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("Joystick Status")
        self.root.geometry("300x250")
        
        self.label_text_1 = tk.StringVar(value="Joystick X: ")
        self.label_text_2 = tk.StringVar(value="Joystick Y: ")
        self.label_text_3 = tk.StringVar(value="Buttons Joy: ")
        # self.label_text_4 = tk.StringVar(value="Joystick EZ: ")
        
        self.label_1 = tk.Label(self.root, textvariable=self.label_text_1, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=0, column=0)
        self.label_2 = tk.Label(self.root, textvariable=self.label_text_2, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=1, column=0)
        self.label_3 = tk.Label(self.root, textvariable=self.label_text_3, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=2, column=0)
        # self.label_4 = tk.Label(self.root, textvariable=self.label_text_4, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=3, column=0)
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=4, column=0)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
        
    def listener_callback(self, msg: Joy):
        self.label_text_1.set(f'Joystick X: "{msg.axes[0]:.2f}"')
        self.label_text_2.set(f'Joystick Y: "{msg.axes[1]:.2f}"')
        self.label_text_3.set(f'Buttons Joy: "{msg.buttons[0]}"')
        # self.label_text_4.set(f'Joystick EZ: "{msg.buttons[0]}"')
        # Print joystick and button data
        self.get_logger().info(
        f'Joystick: X={msg.axes[0]:.2f}, Y={msg.axes[1]:.2f} | '
        f'Buttons: Joy={msg.buttons[0]}'
        )
     
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = JoystickControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy

# class JoystickControlGUI(Node):
#     def __init__(self):
#         super().__init__('joystick_control_gui')
#         self.subscription = self.create_subscription(
#             Joy,
#             'joystick_input',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning
        
#     def listener_callback(self, msg):
#         # Print joystick and button data
#         self.get_logger().info(
#             f'Joystick: X={msg.axes[0]:.2f}, Y={msg.axes[1]:.2f} | '
#             f'Buttons: Joy={msg.buttons[0]}, EZ={msg.buttons[1]}'
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     joystick_listener = JoystickControlGUI()
#     rclpy.spin(joystick_listener)
#     joystick_listener.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()