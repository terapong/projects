#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

# from std_msgs.msg import Int32
from sensor_msgs.msg import Range

class UltrasonicControlGUI(Node):
    def __init__(self, root):
        super().__init__('ultrasonic_control_gui')
        
        self.subscription = self.create_subscription(Range, 'ultrasonic_distance', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("Ultrasonic Status")
        self.root.geometry("600x200")
        
        self.label_text_1 = tk.StringVar(value="Distance: ")
        self.label_text_2 = tk.StringVar(value="min: ")
        self.label_text_3 = tk.StringVar(value="max: ")
        
        self.label_1 = tk.Label(self.root, textvariable=self.label_text_1, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=0, column=0)
        self.label_2 = tk.Label(self.root, textvariable=self.label_text_2, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=1, column=0)
        self.label_3 = tk.Label(self.root, textvariable=self.label_text_3, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=2, column=0)
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=4, column=0, columnspan=2)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
    
    def listener_callback(self, msg):
        # self.label_text_1.set(f'Distance value: "{msg.range}"  cm')
        # self.get_logger().info(f'Distance: {msg.range} cm')
        self.label_text_1.set(f'Distance: "{msg.range}"  cm')
        self.label_text_2.set(f'min: "{msg.min_range}"')
        self.label_text_3.set(f'max: "{msg.max_range}"')
        print(f"Range: {msg.range:.2f} m (min: {msg.min_range}, max: {msg.max_range})")
     
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = UltrasonicControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()