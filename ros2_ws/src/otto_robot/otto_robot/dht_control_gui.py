#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from sensor_msgs.msg import Temperature, RelativeHumidity

class DHTControlGUI(Node):
    def __init__(self, root):
        super().__init__('dht_control_gui')
        
        self.temp_subscription = self.create_subscription(Temperature, 'esp32/temperature', self.temp_callback, 10)
        self.hum_subscription = self.create_subscription(RelativeHumidity, 'esp32/humidity', self.hum_callback, 10)
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("Relay Control")
        self.root.geometry("650x150")
        
        self.label_Temperature = tk.StringVar(value="Temperature : ")
        self.label_Humidity = tk.StringVar(value="Humidity        : ")
        
        self.labelTemperature = tk.Label(self.root, textvariable=self.label_Temperature, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=0, column=0)  
        self.labelHumidity = tk.Label(self.root, textvariable=self.label_Humidity, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=1, column=0)  
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=2, column=0)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
    
    def temp_callback(self, msg):             
        self.label_Temperature.set(f'Temperature : "{msg.temperature}" °C')
        self.get_logger().info(f'Temperature : {msg.temperature} °C')
        
    def hum_callback(self, msg):
        self.label_Humidity.set(f'Humidity    : "{msg.relative_humidity}" %')
        self.get_logger().info(f'Humidity    : {msg.relative_humidity} %')
        
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = DHTControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main().grid(row=0, column=0)