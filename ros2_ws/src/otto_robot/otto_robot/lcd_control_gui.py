#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import String
from std_msgs.msg import Bool

class LCDControlGUI(Node):
    def __init__(self, root):
        super().__init__('lcd_control_gui')
        
        self.publisher = self.create_publisher(String, '/lcd_display_topic', 10)
        self.subscription = self.create_subscription(String, '/lcd_display_topic', self.listener_callback, 10)  
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("LCD Control")
        self.root.geometry("400x200")
        
        self.entryStringt = tk.StringVar()
        
        self.label = tk.Label(self.root, text="Text: ", fg="red", bg="yellow", font="Calibri 15 bold").grid(row=0, column=0)
        self.entry = tk.Entry(self.root, font="Calibri 15 bold", textvariable=self.entryStringt).grid(row=0, column=1)        
        self.button = tk.Button(self.root, text="Input", command=self.selectNumber).grid(row=0, column=2)
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=4, column=0)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
        
    def selectNumber(self):
        msg = String()
        msg.data = self.entryStringt.get()
        self.publisher.publish(msg)              
        self.get_logger().info("Relay ON message published.")
    
    def listener_callback(self, msg):
        self.entryStringt.set(f'Text: "{msg.data}"')
        # self.label.config(text=f'Text: "{msg.data}"')
        
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = LCDControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()