#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import Int32

class SERVOControlGUI(Node):
    def __init__(self, root):
        super().__init__('servo_control_gui')
        
        self.publisher = self.create_publisher(Int32, 'servo_angle', 10)
        self.subscription = self.create_subscription(Int32, 'actual_servo_angle', self.actual_angle_callback, 10)
        self.subscription  # prevent unused variable warning
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("Servo angle Control")
        self.root.geometry("300x150")
        
        self.sc = tk.Scale(self.root, from_=0, to=180,orient='horizontal',length=160)
        self.sc.grid(row=1, column=1,padx=20)
        self.sc.pack()
        
        self.bt = tk.Button(self.root, text="Servo", command=self.set_angle)
        self.bt.pack()
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close)
        self.close_btn.pack()
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
    
    def set_angle(self):
        msg = Int32()
        msg.data = self.sc.get()
        self.publisher.publish(msg)
        self.get_logger().info(f'Setting servo angle to: {msg.data}°')
        
    def actual_angle_callback(self, msg):
        self.get_logger().info(f'Actual servo angle: {msg.data}°')
    
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = SERVOControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

