#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import Int32

class water_sensorControlGUI(Node):
    def __init__(self, root):
        super().__init__('water_sensor_control_gui')
        
        self.subscription = self.create_subscription(Int32, 'water_level', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("water sensor Control")
        self.root.geometry("450x90")
        
        self.entryStringt = tk.StringVar()
        self.label_1 = tk.Label(self.root, textvariable=self.entryStringt, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=0, column=0)  
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=1, column=0)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
        self.update_gui()
        
    def update_gui(self):
        self.root.update()
        
    def listener_callback(self, msg):
        water_level = msg.data
        self.entryStringt.set(f'Current water level: "{water_level}"')
        self.get_logger().info(f'Current water level: {water_level}')
        
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = water_sensorControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()