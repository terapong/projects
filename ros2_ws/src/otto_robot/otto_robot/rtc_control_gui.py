#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import String
# from std_msgs.msg import Bool

class RTCControlGUI(Node):
    def __init__(self, root):
        super().__init__('rtc_control_gui')
        
        self.subscription = self.create_subscription(String, 'rtc_time', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning  
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("RTC Control")
        self.root.geometry("670x100")
        
        self.rtc = tk.StringVar(value="Received RTC time: ")
        
        self.label = tk.Label(self.root, textvariable=self.rtc, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=0, column=0)
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=4, column=0)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
        
    def listener_callback(self, msg):
        # self.label.config(text=f'Received RTC time: "{msg.data}"')
        self.rtc.set(f'Received RTC time: "{msg.data}"')
        self.get_logger().info(f'Received RTC time: {msg.data}')
        
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = RTCControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
