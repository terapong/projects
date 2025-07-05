#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import Int32

class Segment7ControlGUI(Node):
    def __init__(self, root):
        super().__init__('segment7_control_gui')
        
        self.subscription = self.create_subscription(Int32, '/segment7_state', self.listener_callback, 10)      
        self.publisher_ = self.create_publisher(Int32, '/segment7_command', 10)  #led_state
                
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("7 segment Control")
        self.root.geometry("600x120")

        self.entryInt = tk.IntVar()
        
        self.label_1 = tk.Label(self.root, text="Select number", font="Calibri 24 bold").grid(row=0, column=0)      
        self.entry = tk.Entry(self.root, font="Calibri 15 bold", textvariable=self.entryInt).grid(row=0, column=1)
        self.button = tk.Button(self.root, text="Input number", command=self.selectNumber).grid(row=0, column=2)
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=1, column=1)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
        
    def selectNumber(self):
        try:
            msg = Int32() # Bool()
            msg.data = self.entryInt.get()
            self.publisher_.publish(msg)              
        except Exception:
            self.entryInt.set(10)
            msg = Int32() # Bool()
            msg.data = self.entryInt.get()
            self.publisher_.publish(msg)                 
    
    def listener_callback(self, msg):
        # self.label.config(text=f'7 segment: "{msg.data}"')
        self.get_logger().info(f'7 segment: {msg.data}')
        
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = Segment7ControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()