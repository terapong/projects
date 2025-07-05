#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import Bool

class RelayControlGUI(Node):
    def __init__(self, root):
        super().__init__('relay_control_gui')
        
        self.subscription = self.create_subscription(Bool, '/relay_state', self.listener_callback, 10)    
        self.publisher_ = self.create_publisher(Bool, '/relay_command', 10)  #led_state
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("Relay Control")
        self.root.geometry("250x150")
        
        self.label_text_1 = tk.StringVar(value="Relay: ")
        
        self.label_1 = tk.Label(self.root, textvariable=self.label_text_1, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=0, column=0)

        self.on_button = tk.Button(self.root, text="Turn ON", command=self.turn_on).grid(row=1, column=0)
        self.off_button = tk.Button(self.root, text="Turn OFF", command=self.turn_off).grid(row=2, column=0)
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=4, column=0)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
    
    def turn_on(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)              

    def turn_off(self):
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)      
        
    def listener_callback(self, msg):
        self.label_text_1.set(f'Relay: "{msg.data}"')
        self.get_logger().info(f'Relay: {msg.data}')
     
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = RelayControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()