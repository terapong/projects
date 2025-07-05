#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import Int32

class BuzzerControlGUI(Node):
    def __init__(self, root):
        super().__init__('buzzer_control_gui')
        
        self.publisher = self.create_publisher(Int32, 'buzzer_control', 10)
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("Buzzer Control")
        self.root.geometry("300x130")
        
        self.entryInt_frequency = tk.IntVar()
        self.entryInt_duration = tk.IntVar()
        
        self.label_frequency = tk.Label(self.root, text="Beeping number").grid(row=0, column=0)
        self.entry_frequency = tk.Entry(self.root, text="100", textvariable=self.entryInt_frequency).grid(row=0, column=1)#ทำไงให้ใส่แค่ เลข 0-xxx
        
        self.label_duration = tk.Label(self.root, text="Stop time").grid(row=1, column=0)
        self.entry_duration = tk.Entry(self.root, text="1000", textvariable=self.entryInt_duration).grid(row=1, column=1)#ทำไงให้ใส่แค่ เลข 0-xxx
        
        self.button_beep = tk.Button(self.root, text="Beeping", command=self.beep).grid(row=2, column=0)
        self.button_stop = tk.Button(self.root, text="Stop", command=self.stop).grid(row=2, column=1)
        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=3, column=0, columnspan=2)  

        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
    
    def beep(self, frequency=100, duration=1000):
        if self.entryInt_frequency.get() > 0:
            frequency = self.entryInt_frequency.get()
            duration = self.entryInt_duration.get()
        # else:
        #     frequency=0
        #     duration=0

        msg = Int32()
        msg.data = frequency
        self.publisher.publish(msg)
        # Schedule a stop after duration
        self.create_timer(duration / 100, lambda: self.stop())
        self.get_logger().info(f'Beeping at {self.entryInt_frequency.get()}Hz')
        
    def stop(self):
        msg = Int32()
        msg.data = 0
        self.publisher.publish(msg)
        self.get_logger().info('Buzzer stopped')
     
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.stop()
            self.root.destroy()
            self.root.mainloop()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = BuzzerControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()