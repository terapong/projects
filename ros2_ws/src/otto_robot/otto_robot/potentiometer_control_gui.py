#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import Int32

class PotentiometerControlGUI(Node):
    def __init__(self, root):
        super().__init__('potentiometer_control_gui')
        
        self.subscription = self.create_subscription(Int32, 'potentiometer_value', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("Potentiometer Status")
        self.root.geometry("460x150")
        
        self.label_text_1 = tk.StringVar(value="Potentiometer value: ")
        self.label_text_2 = tk.StringVar(value="Voltage: ") 
        
        self.label_1 = tk.Label(self.root, textvariable=self.label_text_1, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=0, column=0)
        self.label_2 = tk.Label(self.root, textvariable=self.label_text_2, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=1, column=0)

        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=2, column=0, columnspan=2)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
    
    def listener_callback(self, msg):
        voltage = (msg.data / 16)
        self.get_logger().info(f'Potentiometer value: {msg.data} (Voltage: {voltage:.2f}V)')
        # light_value = msg.data
        # Convert ADC value to percentage (adjust based on your LDR's range)
        # percentage = min(100, max(0, (light_value - 500) / (3000 - 500) * 100))
        
        # self.get_logger().info(f'Light level: {light_value} (ADC), {percentage:.1f}%') 
        
        self.label_text_1.set(f'Potentiometer value: "{msg.data}"')
        self.label_text_2.set(f'            Voltage: "{voltage:.2f}"')
        
#     def listener_callback(self, msg):
#         # Convert ADC value to voltage (assuming 3.3V reference and 12-bit ADC)
#         voltage = (msg.data / 4095) * 3.3
#         self.get_logger().info(f'Potentiometer value: {msg.data} (Voltage: {voltage:.2f}V)')
     
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = PotentiometerControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()