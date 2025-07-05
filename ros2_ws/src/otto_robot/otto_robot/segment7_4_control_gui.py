#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import Int32

class Segment74ControlGUI(Node):
    def __init__(self, root):
        super().__init__('segment7_4_control_gui')
        
        # self.subscription = self.create_subscription(Int32, '/segment7_4_state', self.listener_callback, 10)      
        # self.publisher_ = self.create_publisher(Int32, '/segment7_4_command', 10)  #led_state
        
        self.publisher = self.create_publisher(Int32, 'display_value', 10)
        self.subscription = self.create_subscription(Int32, '/display_value', self.listener_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_value)
        self.counter = 0
                
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("7 4 segment Control")
        self.root.geometry("600x120")
        
        self.label_text_1 = tk.StringVar(value="7 4 segment Control: ")
        self.label_1 = tk.Label(self.root, textvariable=self.label_text_1, fg="red", bg="yellow", font="Calibri 24 bold").grid(row=0, column=0)
        
        # self.label_1 = tk.Label(self.root, text="Select number: ", font="Calibri 24 bold").grid(row=0, column=0)      
        # self.entry = tk.Entry(self.root, font="Calibri 15 bold", textvariable=self.entryInt).grid(row=0, column=1)
        # self.button = tk.Button(self.root, text="Input 4 number", command=self.selectNumber).grid(row=0, column=2)
                
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=1, column=0)
        
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
        
    # def selectNumber(self):
    #     try:
    #         msg = Int32() # Bool()
    #         msg.data = self.entryInt.get()
    #         self.publisher_.publish(msg)              
    #     except Exception:
    #         self.entryInt.set(10)
    #         msg = Int32() # Bool()
    #         msg.data = self.entryInt.get()
    #         self.publisher_.publish(msg)    
            
    def publish_value(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1
        if self.counter > 9999:
            self.counter = 0              
    
    def listener_callback(self, msg):
        self.label_text_1.set(f'7 4 segment Control: "{msg.data}"')
        self.get_logger().info(f'7 4 segment Control: {msg.data}')
        
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
            self.root.quit()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = Segment74ControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32

# class Segment74ControlGUI(Node):
#     def __init__(self):
#         super().__init__('segment7_4_control_gui')
#         self.publisher = self.create_publisher(Int32, 'display_value', 10)
#         self.timer = self.create_timer(1.0, self.publish_value)
#         self.counter = 0
    
#     def publish_value(self):
#         msg = Int32()
#         msg.data = self.counter
#         self.publisher.publish(msg)
#         self.get_logger().info(f'Publishing: {msg.data}')
#         self.counter += 1
#         if self.counter > 9999:
#             self.counter = 0

# def main(args=None):
#     rclpy.init(args=args)
#     node = Segment74ControlGUI()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
    
